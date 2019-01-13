#[macro_use]
extern crate log;

use nalgebra as na;
use ncollide2d::math::{Isometry as Isometry2, Translation as Translation2, Vector as Vector2};
use ncollide2d::shape::{Ball as Ball2, ShapeHandle};
use ncollide2d::world::{
    CollisionGroups, CollisionObjectHandle, CollisionWorld, GeometricQueryType,
};

use quicksilver::{
    geom::{Circle, Rectangle, Vector},
    graphics::{Background::Col, Color},
    lifecycle::{run, Settings, State, Window},
    //input::Key,
    Result,
};

struct BallPos(Vector2<f32>);

impl Into<Vector> for BallPos {
    fn into(self) -> Vector {
        Vector::new(self.0.x, self.0.y)
    }
}

#[derive(Clone)]
struct CollisionObjectData {
    pub name: &'static str,
    pub velocity: Option<Vector2<f32>>,
}

impl CollisionObjectData {
    pub fn new(name: &'static str, velocity: Option<Vector2<f32>>) -> CollisionObjectData {
        CollisionObjectData { name, velocity }
    }
}

struct PoolTable {
    width: f32,
    height: f32,
    margin_left: f32,
    margin_top: f32,
    padding: f32,
    hole_size: f32,
    world: CollisionWorld<f32, CollisionObjectData>,
    white_ball_handle: CollisionObjectHandle,
}

impl State for PoolTable {
    fn new() -> Result<PoolTable> {
        let width = 772.;
        let height = 400.;
        let margin_left = 100.;
        let margin_top = 150.;
        let padding = 30.;
        let hole_size = 16.;
        let mut world = CollisionWorld::new(0.02);
        let ball_size = 12.0_f32;

        let ball_shape = ShapeHandle::new(Ball2::new(ball_size));
        let white_ball = CollisionObjectData::new("white_ball", Some(Vector2::new(50., 70.)));
        let white_pos = Isometry2::new(
            Vector2::new(
                (margin_left + padding + width) as f32 / 4.,
                (margin_top + padding + height) as f32 / 2.,
            ),
            na::zero(),
        );

        let balls_groups = CollisionGroups::new();
        let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);

        let white_ball_handle = world.add(
            white_pos,
            ball_shape,
            balls_groups,
            contacts_query,
            white_ball.clone(),
        );

        Ok(PoolTable {
            width,
            height,
            margin_left,
            margin_top,
            padding,
            hole_size,
            world: world,
            white_ball_handle: white_ball_handle,
        })
    }

    fn draw(&mut self, window: &mut Window) -> Result<()> {
        window.clear(Color::WHITE)?;

        let green = Color::WHITE
            .with_red(0x28 as f32 / 0xff as f32)
            .with_green(0x6b as f32 / 0xff as f32)
            .with_blue(0x31 as f32 / 0xff as f32);
        let band = Color::WHITE
            .with_red(0x0b as f32 / 0xff as f32)
            .with_green(0x45 as f32 / 0xff as f32)
            .with_blue(0x16 as f32 / 0xff as f32);
        let table = Color::WHITE
            .with_red(0x4a as f32 / 0xff as f32)
            .with_green(0x2c as f32 / 0xff as f32)
            .with_blue(0x14 as f32 / 0xff as f32);

        window.draw(
            &Rectangle::new(
                (self.margin_left, self.margin_top),
                (
                    self.width + self.padding * 2.,
                    self.height + self.padding * 2.,
                ),
            ),
            Col(table),
        );
        window.draw(
            &Rectangle::new(
                (
                    self.margin_left + self.padding - self.hole_size / 2.,
                    self.margin_top + self.padding - self.hole_size / 2.,
                ),
                (self.width + self.hole_size, self.height + self.hole_size),
            ),
            Col(band),
        );
        window.draw(
            &Rectangle::new(
                (
                    self.margin_left + self.padding,
                    self.margin_top + self.padding,
                ),
                (self.width, self.height),
            ),
            Col(green),
        );

        window.draw(
            &Circle::new(
                (
                    self.margin_left + self.padding,
                    self.margin_top + self.padding,
                ),
                self.hole_size,
            ),
            Col(Color::BLACK),
        );
        window.draw(
            &Circle::new(
                (
                    self.margin_left + self.padding + self.width / 2.,
                    self.margin_top + self.padding - self.hole_size / 3.,
                ),
                self.hole_size,
            ),
            Col(Color::BLACK),
        );
        window.draw(
            &Circle::new(
                (
                    self.margin_left + self.padding + self.width,
                    self.margin_top + self.padding,
                ),
                self.hole_size,
            ),
            Col(Color::BLACK),
        );

        window.draw(
            &Circle::new(
                (
                    self.margin_left + self.padding,
                    self.margin_top + self.padding + self.height,
                ),
                self.hole_size,
            ),
            Col(Color::BLACK),
        );
        window.draw(
            &Circle::new(
                (
                    self.margin_left + self.padding + self.width / 2.,
                    self.margin_top + self.padding + self.height + self.hole_size / 3.,
                ),
                self.hole_size,
            ),
            Col(Color::BLACK),
        );
        window.draw(
            &Circle::new(
                (
                    self.margin_left + self.padding + self.width,
                    self.margin_top + self.padding + self.height,
                ),
                self.hole_size,
            ),
            Col(Color::BLACK),
        );

        //self.white_ball.draw(window);
        let ball_object = self.world.collision_object(self.white_ball_handle).unwrap();
        let pos = ball_object.position().clone();
        let ball_ball: &Ball2<f32> = ball_object.shape().as_shape().unwrap();

        let pos = pos.translation.vector;

        window.draw(
            &Circle::from_ball(BallPos(pos), ball_ball.clone()),
            Col(Color::WHITE),
        );

        Ok(())
    }

    fn update(&mut self, _window: &mut Window) -> Result<()> {
        let timestep = 0.06;
        {
            let ball_object = self.world.collision_object(self.white_ball_handle).unwrap();
            let ball_velocity = ball_object.data().velocity.as_ref();

            if let Some(vel) = ball_velocity {
                // Integrate the positions.
                let displacement = Translation2::from(timestep * vel);
                let ball_pos = displacement * ball_object.position();

                self.world.set_position(self.white_ball_handle, ball_pos);
            }
        }
        let ball_object = self
            .world
            .collision_object_mut(self.white_ball_handle)
            .unwrap();
        let ball = ball_object.data_mut();
        if let Some(vel) = ball.velocity.as_ref() {
            let new_vel = Vector2::new(vel.x * 0.99, vel.y * 0.99);
            if new_vel.x > 0.001 && new_vel.y > 0.001 {
                ball.velocity = Some(new_vel);
            } else {
                ball.velocity = None;
            }
        }

        Ok(())
    }
}

fn main() {
    info!("here");
    run::<PoolTable>("Draw Table", Vector::new(1024, 768), Settings::default());
    info!("there");
}
