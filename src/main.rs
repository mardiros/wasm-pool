#[macro_use]
extern crate log;
use std::cell::Cell;

use nalgebra as na;
use ncollide2d::events::ContactEvent;
use ncollide2d::math::{Isometry as Isometry2, Translation as Translation2, Vector as Vector2};
use ncollide2d::shape::{Ball, Plane, ShapeHandle};
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
    pub velocity: Option<Cell<Vector2<f32>>>,
}

impl CollisionObjectData {
    pub fn new(name: &'static str, velocity: Option<Vector2<f32>>) -> CollisionObjectData {
        let init_velocity;
        if let Some(velocity) = velocity {
            init_velocity = Some(Cell::new(velocity))
        } else {
            init_velocity = None
        }

        CollisionObjectData {
            name: name,
            velocity: init_velocity,
        }
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
        let ball_size = 12.0_f32;

        let ball_shape = ShapeHandle::new(Ball::new(ball_size));
        let white_ball = CollisionObjectData::new("white_ball", Some(Vector2::new(90., 70.)));
        let white_pos = Isometry2::new(
            Vector2::new(
                (margin_left + padding + width) as f32 * 0.28,
                (margin_top + padding + height) as f32 * 0.5,
            ),
            na::zero(),
        );

        let mut balls_groups = CollisionGroups::new();
        balls_groups.set_membership(&[1]);
        let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);

        let mut band_groups = CollisionGroups::new();
        band_groups.set_membership(&[2]);
        band_groups.set_whitelist(&[1]);

        let band = CollisionObjectData::new("band", None);
        let band_left = ShapeHandle::new(Plane::new(Vector2::x_axis()));
        let band_bottom = ShapeHandle::new(Plane::new(Vector2::y_axis()));
        let band_right = ShapeHandle::new(Plane::new(-Vector2::x_axis()));
        let band_top = ShapeHandle::new(Plane::new(-Vector2::y_axis()));

        // Positions of the planes.
        let bands_pos = [
            Isometry2::new(Vector2::new(margin_left + padding, 0.0), na::zero()),
            Isometry2::new(Vector2::new(0.0, margin_top + padding), na::zero()),
            Isometry2::new(Vector2::new(margin_left + padding + width, 0.0), na::zero()),
            Isometry2::new(Vector2::new(0.0, margin_top + padding + height), na::zero()),
        ];

        let mut world = CollisionWorld::new(0.02);

        let _handle0 = world.add(
            bands_pos[0],
            band_left,
            band_groups,
            contacts_query,
            band.clone(),
        );
        let _handle1 = world.add(
            bands_pos[1],
            band_bottom,
            band_groups,
            contacts_query,
            band.clone(),
        );
        let _handle2 = world.add(
            bands_pos[2],
            band_right,
            band_groups,
            contacts_query,
            band.clone(),
        );

        let _handle3 = world.add(
            bands_pos[3],
            band_top,
            band_groups,
            contacts_query,
            band.clone(),
        );

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
        let background = Color::WHITE
            .with_red(0xcc as f32 / 0xff as f32)
            .with_green(0xcc as f32 / 0xff as f32)
            .with_blue(0xcc as f32 / 0xff as f32);

        window.clear(background)?;

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
        let ball_ball: &Ball<f32> = ball_object.shape().as_shape().unwrap();

        let pos = pos.translation.vector;

        window.draw(
            &Circle::from_ball(BallPos(pos), ball_ball.clone()),
            Col(Color::WHITE),
        );

        Ok(())
    }

    fn update(&mut self, _window: &mut Window) -> Result<()> {
        self.update_position();
        self.update_vellocity();
        self.world.update();

        for event in self.world.contact_events() {
            self.handle_contact_event(event)
        }
        Ok(())
    }
}

impl PoolTable {
    fn handle_contact_event(&self, event: &ContactEvent) {
        if let &ContactEvent::Started(collider1, collider2) = event {
            // NOTE: real-life applications would avoid this systematic allocation.
            let pair = self.world.contact_pair(collider1, collider2).unwrap();
            let mut collector = Vec::new();
            pair.contacts(&mut collector);

            let co1 = self.world.collision_object(collider1).unwrap();
            let co2 = self.world.collision_object(collider2).unwrap();

            //  balls has velocity, border don't.
            if let Some(ref vel) = co1.data().velocity {
                let normal = collector[0].deepest_contact().unwrap().contact.normal;
                vel.set(vel.get() - 2.0 * na::dot(&vel.get(), &normal) * *normal)
            }
            if let Some(ref vel) = co2.data().velocity {
                let normal = collector[0].deepest_contact().unwrap().contact.normal;
                let new_vel = vel.get() - 2.0 * na::dot(&vel.get(), &normal) * *normal;
                vel.set(new_vel)
            }
        }
    }

    fn update_position(&mut self) {
        let timestep = 0.06;
        let ball_object = self.world.collision_object(self.white_ball_handle).unwrap();
        let ball_velocity = ball_object.data().velocity.as_ref();

        if let Some(vel) = ball_velocity {
            // Integrate the positions.
            let displacement = Translation2::from(timestep * vel.get());
            let ball_pos = displacement * ball_object.position();

            self.world.set_position(self.white_ball_handle, ball_pos);
            //info!("Ball position: {:?}", ball_pos)
        }
    }
    fn update_vellocity(&mut self) {
        let ball_object = self
            .world
            .collision_object_mut(self.white_ball_handle)
            .unwrap();
        let ball = ball_object.data_mut();
        if let Some(ref vel) = ball.velocity {
            let new_vel = Vector2::new(vel.get().x * 0.99, vel.get().y * 0.99);
            if new_vel.x < -10. || new_vel.x > 10. || new_vel.y > 10. || new_vel.y < -10. {
                if let Some(ref vel) = ball.velocity {
                    vel.set(new_vel);
                }
            } else {
                ball.velocity = None;
            }
        }
    }
}
fn main() {
    web_logger::init();
    info!("Starting the pool");
    run::<PoolTable>("PoolTable", Vector::new(1024, 768), Settings::default());
    info!("Started");
}
