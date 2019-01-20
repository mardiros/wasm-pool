#[macro_use]
extern crate log;

use std::vec::Vec;

use nalgebra as na;
use ncollide2d::events::ContactEvent;
use ncollide2d::math::{Isometry as Isometry2, Vector as Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::force_generator::ForceGenerator;
use nphysics2d::math::Velocity;
use nphysics2d::object::{BodyHandle, BodySet, Material};
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;

use quicksilver::{
    geom::{Circle, Rectangle, Vector},
    graphics::{Background::Col, Color},
    lifecycle::{run, Settings, State, Window},
    //input::Key,
    Result,
};

const COLLIDER_MARGIN: f32 = 0.01;
const BALL_SIZE: f32 = 12.0;
const WIDTH: f32 = 772.0;
const HEIGHT: f32 = 400.;
const MARGIN_TOP: f32 = 150.;
const MARGIN_LEFT: f32 = 100.;
const BORDER: f32 = 28.;
const BAND: f32 = 8.;
const HOLE_SIZE: f32 = 16.;
const Z_GRAVITY: f32 = -0.81;

pub struct ZGravity {
    parts: Vec<BodyHandle>, // Body parts affected by the force generator.
}

impl ZGravity {
    // Creates a new radial force generator.
    pub fn new(parts: Vec<BodyHandle>) -> Self {
        ZGravity { parts }
    }

    /// Add a body part to be affected by this force generator.
    pub fn add_body_part(&mut self, body: BodyHandle) {
        self.parts.push(body)
    }
}

impl ForceGenerator<f32> for ZGravity {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        let mut i = 0;

        while i < self.parts.len() {
            let body = self.parts[i];
            if bodies.contains(body) {
                let mut part = bodies.body_part_mut(body);
                let mut vel = part.as_ref().velocity();
                vel.linear.x = vel.linear.x * Z_GRAVITY;
                vel.linear.y = vel.linear.y * Z_GRAVITY;
                let force = part.as_ref().inertia() * vel;
                part.apply_force(&force);
                i += 1;
            } else {
                let _ = self.parts.swap_remove(i);
            }
        }
        true
    }
}

struct FromNPVec(Vector2<f32>);

impl Into<Vector> for FromNPVec {
    fn into(self) -> Vector {
        Vector::new(self.0.x, self.0.y)
    }
}

struct PoolTable {
    world: World<f32>,
    white_ball_handle: BodyHandle,
    ball_8_handle: BodyHandle,
}

impl State for PoolTable {
    fn new() -> Result<PoolTable> {
        let mut world: World<f32> = World::new();

        let material: Material<f32> = Material::new(0.5, 0.);
        let ball_material: Material<f32> = Material::new(0.94, 0.);

        let height_border_shape: ShapeHandle<f32> = ShapeHandle::new(Cuboid::new(Vector2::new(
            COLLIDER_MARGIN,
            HEIGHT + 2. * BORDER - 2. * COLLIDER_MARGIN,
        )));
        let width_border_shape: ShapeHandle<f32> = ShapeHandle::new(Cuboid::new(Vector2::new(
            WIDTH + 2. * BORDER - 2. * COLLIDER_MARGIN,
            COLLIDER_MARGIN,
        )));

        let top = MARGIN_TOP + BORDER;
        let left = MARGIN_LEFT + BORDER;
        // left
        let border_pos = Isometry2::new(Vector2::new(left, top), na::zero());
        world.add_collider(
            COLLIDER_MARGIN,
            height_border_shape.clone(),
            BodyHandle::ground(),
            border_pos,
            material.clone(),
        );

        // top
        let border_pos = Isometry2::new(Vector2::new(left, top), na::zero());
        world.add_collider(
            COLLIDER_MARGIN,
            width_border_shape.clone(),
            BodyHandle::ground(),
            border_pos,
            material.clone(),
        );

        // right
        let border_pos = Isometry2::new(Vector2::new(left + WIDTH + 2. * BAND, top), na::zero());
        world.add_collider(
            COLLIDER_MARGIN,
            height_border_shape.clone(),
            BodyHandle::ground(),
            border_pos,
            material.clone(),
        );
        // bottom
        let border_pos = Isometry2::new(Vector2::new(left, top + HEIGHT + 2. * BAND), na::zero());
        world.add_collider(
            COLLIDER_MARGIN,
            width_border_shape.clone(),
            BodyHandle::ground(),
            border_pos,
            material.clone(),
        );

        let ball_shape = ShapeHandle::new(Ball::new(BALL_SIZE));
        let white_pos = Isometry2::new(
            Vector2::new(MARGIN_LEFT + WIDTH * 0.28, MARGIN_TOP + HEIGHT * 0.5),
            na::zero(),
        );

        let inertia = ball_shape.inertia(1.);
        let center_of_mass = ball_shape.center_of_mass();
        let white_ball_handle = world.add_rigid_body(white_pos, inertia, center_of_mass);

        world.add_collider(
            COLLIDER_MARGIN,
            ball_shape.clone(),
            white_ball_handle,
            Isometry2::identity(),
            ball_material.clone(),
        );

        let ball_8_pos = Isometry2::new(
            Vector2::new(MARGIN_LEFT + WIDTH * 0.28 + 5., MARGIN_TOP + HEIGHT * 0.75),
            na::zero(),
        );

        let inertia = ball_shape.inertia(1.);
        let center_of_mass = ball_shape.center_of_mass();
        let ball_8_handle = world.add_rigid_body(ball_8_pos, inertia, center_of_mass);

        world.add_collider(
            COLLIDER_MARGIN,
            ball_shape.clone(),
            ball_8_handle,
            Isometry2::identity(),
            ball_material.clone(),
        );

        let ball_object = world.rigid_body_mut(white_ball_handle).unwrap();
        //ball_object.set_status(BodyStatus::Dynamic);
        let vel = Velocity::linear(5.0, 500.0);
        ball_object.set_velocity(vel);

        let mut z_gravity: ZGravity = ZGravity::new(Vec::new());
        z_gravity.add_body_part(white_ball_handle);
        z_gravity.add_body_part(ball_8_handle);
        world.add_force_generator(z_gravity);

        Ok(PoolTable {
            world: world,
            white_ball_handle: white_ball_handle,
            ball_8_handle: ball_8_handle,
        })
    }

    fn draw(&mut self, window: &mut Window) -> Result<()> {
        let background = Color::WHITE
            .with_red(0xcc as f32 / 0xff as f32)
            .with_green(0xcc as f32 / 0xff as f32)
            .with_blue(0xcc as f32 / 0xff as f32);

        window.clear(background)?;

        let table_color = Color::WHITE
            .with_red(0x28 as f32 / 0xff as f32)
            .with_green(0x6b as f32 / 0xff as f32)
            .with_blue(0x31 as f32 / 0xff as f32);
        let band_color = Color::WHITE
            .with_red(0x0b as f32 / 0xff as f32)
            .with_green(0x45 as f32 / 0xff as f32)
            .with_blue(0x16 as f32 / 0xff as f32);
        let border_color = Color::WHITE
            .with_red(0x4a as f32 / 0xff as f32)
            .with_green(0x2c as f32 / 0xff as f32)
            .with_blue(0x14 as f32 / 0xff as f32);
        let hole_color = Color::WHITE
            .with_red(0x22 as f32 / 0xff as f32)
            .with_green(0x22 as f32 / 0xff as f32)
            .with_blue(0x22 as f32 / 0xff as f32);

        window.draw(
            &Rectangle::new(
                (MARGIN_LEFT, MARGIN_TOP),
                (
                    WIDTH + BORDER * 2. + BAND * 2.,
                    HEIGHT + BORDER * 2. + BAND * 2.,
                ),
            ),
            Col(border_color),
        );
        window.draw(
            &Rectangle::new(
                (MARGIN_LEFT + BORDER, MARGIN_TOP + BORDER),
                (WIDTH + BAND * 2., HEIGHT + BAND * 2.),
            ),
            Col(band_color),
        );
        window.draw(
            &Rectangle::new(
                (MARGIN_LEFT + BORDER + BAND, MARGIN_TOP + BORDER + BAND),
                (WIDTH, HEIGHT),
            ),
            Col(table_color),
        );

        // TOP LEFT hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT + BORDER + BAND * 0.75,
                    MARGIN_TOP + BORDER + BAND * 0.75,
                ),
                HOLE_SIZE,
            ),
            Col(hole_color),
        );

        // TOP hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT + BORDER + BAND * 0.5 + WIDTH / 2.,
                    MARGIN_TOP + BORDER + BAND * 0.5,
                ),
                HOLE_SIZE,
            ),
            Col(hole_color),
        );

        // top right hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT + BORDER + BAND + WIDTH + BAND * 0.25,
                    MARGIN_TOP + BORDER + BAND * 0.75,
                ),
                HOLE_SIZE,
            ),
            Col(hole_color),
        );

        // right hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT + BORDER + BAND + WIDTH + BAND * 0.5,
                    MARGIN_TOP + BORDER + BAND * 0.75 + HEIGHT / 2.,
                ),
                HOLE_SIZE,
            ),
            Col(hole_color),
        );

        // bottom right hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT + BORDER + BAND + WIDTH + BAND * 0.25,
                    MARGIN_TOP + BORDER + HEIGHT + BAND * 1.25,
                ),
                HOLE_SIZE,
            ),
            Col(hole_color),
        );

        // bottom hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT + BORDER + BAND * 0.5 + WIDTH / 2.,
                    MARGIN_TOP + BORDER + HEIGHT + BAND * 1.5,
                ),
                HOLE_SIZE,
            ),
            Col(hole_color),
        );

        // bottom left hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT + BORDER + BAND * 0.75,
                    MARGIN_TOP + BORDER + HEIGHT + BAND * 1.25,
                ),
                HOLE_SIZE,
            ),
            Col(hole_color),
        );

        // left hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT + BORDER + BAND * 0.25,
                    MARGIN_TOP + BORDER + BAND * 0.75 + HEIGHT / 2.,
                ),
                HOLE_SIZE,
            ),
            Col(hole_color),
        );

        //self.white_ball.draw(window);
        let ball_object = self.world.body_part(self.white_ball_handle);
        let pos = ball_object.position().clone();
        let pos = pos.translation.vector;
        //info!("Ball pos: {:?}", pos);
        let ball_ball = Ball::new(BALL_SIZE);

        window.draw(
            &Circle::from_ball(FromNPVec(pos), ball_ball),
            Col(Color::WHITE),
        );

        let ball_object = self.world.body_part(self.ball_8_handle);
        let pos = ball_object.position().clone();
        let pos = pos.translation.vector;
        //info!("Ball pos: {:?}", pos);
        let ball_ball = Ball::new(BALL_SIZE);

        window.draw(
            &Circle::from_ball(FromNPVec(pos), ball_ball),
            Col(Color::BLACK),
        );

        Ok(())
    }

    fn update(&mut self, _window: &mut Window) -> Result<()> {
        self.world.step();
        for contact in self.world.contact_events() {
            // Handle contact events.
            self.handle_contact_event(contact)
        }
        Ok(())
    }
}

impl PoolTable {
    fn handle_contact_event(&self, event: &ContactEvent) {
        if let &ContactEvent::Started(collider1, collider2) = event {
            info!(
                "!!! handle_contact_event {:?} {:?} {:?}",
                event, collider1, collider2
            );
        }
    }
}

fn main() {
    web_logger::init();
    info!("Starting the pool");
    run::<PoolTable>("PoolTable", Vector::new(1024, 768), Settings::default());
    info!("Started");
}
