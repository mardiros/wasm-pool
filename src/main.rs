#[macro_use]
extern crate log;

use std::vec::Vec;

use nalgebra as na;

use ncollide2d::{
    events::ContactEvent,
    math::{Isometry as Isometry2, Vector as Vector2},
    shape::{Ball, Cuboid, ShapeHandle},
};

use nphysics2d::{
    force_generator::ForceGenerator,
    math::Velocity,
    object::{BodyHandle, BodySet, Material},
    solver::{IntegrationParameters, SignoriniModel},
    volumetric::Volumetric,
    world::World,
};

use quicksilver::{
    geom::{Circle, Rectangle, Transform, Vector},
    graphics::{Background::Col, Color},
    input::Key,
    lifecycle::{run, Settings, State, Window},
    Result,
};

const COLLIDER_MARGIN: f32 = 0.01;
const BALL_SIZE: f32 = 120.0;
const WIDTH: f32 = 7720.0;
const HEIGHT: f32 = 4000.;
const MARGIN_TOP: f32 = 1500.;
const MARGIN_LEFT: f32 = 1000.;
const BORDER: f32 = 280.;
const BAND: f32 = 80.;
const HOLE_SIZE: f32 = 160.;

const WORD_SCALE_FACTOR: f32 = 0.1;

const Z_GRAVITY: f32 = -0.93;

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

fn ball_material() -> Material<f32> {
    Material::new(0.50, 1.)
}

fn ball_shape() -> ShapeHandle<f32> {
    ShapeHandle::new(Ball::new(BALL_SIZE))
}

fn add_ball(x: f32, y: f32, z_gravity: &mut ZGravity, world: &mut World<f32>) -> BodyHandle {
    let ball_shape = ball_shape();
    let ball_material = ball_material();

    let ball_8_pos = Isometry2::new(Vector2::new(x, y), na::zero());

    let inertia = ball_shape.inertia(1.);
    let center_of_mass = ball_shape.center_of_mass();
    let ball_handle = world.add_rigid_body(ball_8_pos, inertia, center_of_mass);

    z_gravity.add_body_part(ball_handle);

    world.add_collider(
        COLLIDER_MARGIN,
        ball_shape.clone(),
        ball_handle,
        Isometry2::identity(),
        ball_material.clone(),
    );
    ball_handle
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

                let inertia = part.as_ref().inertia();
                let force = inertia * vel;
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
        Vector::new(self.0.x * WORD_SCALE_FACTOR, self.0.y * WORD_SCALE_FACTOR)
    }
}

struct PoolTable {
    world: World<f32>,
    white_ball_handle: BodyHandle,
    ball_8_handle: BodyHandle,

    yellow_balls_handles: Vec<BodyHandle>,
    red_balls_handles: Vec<BodyHandle>,

    cane_rotation: f32,
    cane_force: f32,
}

impl State for PoolTable {
    fn new() -> Result<PoolTable> {
        let mut world: World<f32> = World::new();

        let material: Material<f32> = Material::new(0.95, 0.);

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

        // bottom band
        let bound_material: Material<f32> = Material::new(0.3, 1.0);
        let width_band_shape: ShapeHandle<f32> = ShapeHandle::new(Cuboid::new(Vector2::new(
            WIDTH / 2. - 2. * HOLE_SIZE - COLLIDER_MARGIN,
            COLLIDER_MARGIN,
        )));
        let border_pos = Isometry2::new(Vector2::new(left, top + HEIGHT + BAND), na::zero());
        world.add_collider(
            COLLIDER_MARGIN,
            width_band_shape.clone(),
            BodyHandle::ground(),
            border_pos,
            bound_material.clone(),
        );

        let mut z_gravity: ZGravity = ZGravity::new(Vec::new());

        let center_y = MARGIN_TOP + BORDER + HEIGHT * 0.5;
        let white_ball_handle = add_ball(MARGIN_LEFT + BORDER + WIDTH * 0.25, center_y, &mut z_gravity, &mut world);

        let center_x = MARGIN_LEFT + BORDER + WIDTH * 0.75;

        //     r
        let ball_r1 = add_ball(center_x - 4. * BALL_SIZE, center_y,&mut z_gravity,  &mut world);

        //    y r  ( right to left )

        let ball_r2 = add_ball(
            center_x - 2. * BALL_SIZE,
            center_y - 1. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );

        let ball_y1 = add_ball(
            center_x - 2. * BALL_SIZE,
            center_y + 1. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );

        //   r b y  ( right to left )

        let ball_y2 = add_ball(center_x, center_y - 2. * BALL_SIZE,&mut z_gravity,  &mut world);

        let ball_8_handle = add_ball(center_x, center_y,&mut z_gravity,  &mut world);

        let ball_r3 = add_ball(center_x, center_y + 2. * BALL_SIZE,&mut z_gravity,  &mut world);

        //  y r y r  ( right to left )
        let ball_r4 = add_ball(
            center_x + 2. * BALL_SIZE,
            center_y - 3. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );
        let ball_y3 = add_ball(
            center_x + 2. * BALL_SIZE,
            center_y - 1. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );
        let ball_r5 = add_ball(
            center_x + 2. * BALL_SIZE,
            center_y + 1. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );
        let ball_y4 = add_ball(
            center_x + 2. * BALL_SIZE,
            center_y + 3. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );

        // r y r y y ( right to left )
        let ball_y5 = add_ball(
            center_x + 4. * BALL_SIZE,
            center_y - 4. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );
        let ball_y6 = add_ball(
            center_x + 4. * BALL_SIZE,
            center_y - 2. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );
        let ball_r6 = add_ball(center_x + 4. * BALL_SIZE, center_y,&mut z_gravity,  &mut world);
        let ball_y7 = add_ball(
            center_x + 4. * BALL_SIZE,
            center_y + 2. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );
        let ball_r7 = add_ball(
            center_x + 4. * BALL_SIZE,
            center_y + 4. * BALL_SIZE,
           &mut z_gravity,  &mut world,
        );

        let red_balls_handles = vec![
            ball_r1, ball_r2, ball_r3, ball_r4, ball_r5, ball_r6, ball_r7,
        ];
        let yellow_balls_handles = vec![
            ball_y1, ball_y2, ball_y3, ball_y4, ball_y5, ball_y6, ball_y7,
        ];

        world.add_force_generator(z_gravity);

        let model: SignoriniModel<f32> = SignoriniModel::new();
        world.set_contact_model(model);

        let param = world.integration_parameters_mut();
        param.dt = 1. / 120.;

        let cane_rotation = 0.;
        let cane_force = 5.;

        Ok(PoolTable {
            world,
            white_ball_handle,
            ball_8_handle,
            yellow_balls_handles,
            red_balls_handles,
            cane_rotation,
            cane_force,
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
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR,
                    MARGIN_TOP * WORD_SCALE_FACTOR,
                ),
                (
                    WIDTH * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR * 2.
                        + BAND * WORD_SCALE_FACTOR * 2.,
                    HEIGHT * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR * 2.
                        + BAND * WORD_SCALE_FACTOR * 2.,
                ),
            ),
            Col(border_color),
        );
        window.draw(
            &Rectangle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR + BORDER * WORD_SCALE_FACTOR,
                    MARGIN_TOP * WORD_SCALE_FACTOR + BORDER * WORD_SCALE_FACTOR,
                ),
                (
                    WIDTH * WORD_SCALE_FACTOR + BAND * WORD_SCALE_FACTOR * 2.,
                    HEIGHT * WORD_SCALE_FACTOR + BAND * WORD_SCALE_FACTOR * 2.,
                ),
            ),
            Col(band_color),
        );
        window.draw(
            &Rectangle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR,
                    MARGIN_TOP * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR,
                ),
                (WIDTH * WORD_SCALE_FACTOR, HEIGHT * WORD_SCALE_FACTOR),
            ),
            Col(table_color),
        );

        // TOP LEFT hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.75,
                    MARGIN_TOP * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.75,
                ),
                HOLE_SIZE * WORD_SCALE_FACTOR,
            ),
            Col(hole_color),
        );

        // TOP hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.5
                        + WIDTH * WORD_SCALE_FACTOR / 2.,
                    MARGIN_TOP * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.5,
                ),
                HOLE_SIZE * WORD_SCALE_FACTOR,
            ),
            Col(hole_color),
        );

        // top right hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR
                        + WIDTH * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.25,
                    MARGIN_TOP * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.75,
                ),
                HOLE_SIZE * WORD_SCALE_FACTOR,
            ),
            Col(hole_color),
        );

        // bottom right hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR
                        + WIDTH * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.25,
                    MARGIN_TOP * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + HEIGHT * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 1.25,
                ),
                HOLE_SIZE * WORD_SCALE_FACTOR,
            ),
            Col(hole_color),
        );

        // bottom hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.5
                        + WIDTH * WORD_SCALE_FACTOR / 2.,
                    MARGIN_TOP * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + HEIGHT * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 1.5,
                ),
                HOLE_SIZE * WORD_SCALE_FACTOR,
            ),
            Col(hole_color),
        );

        // bottom left hole
        window.draw(
            &Circle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 0.75,
                    MARGIN_TOP * WORD_SCALE_FACTOR
                        + BORDER * WORD_SCALE_FACTOR
                        + HEIGHT * WORD_SCALE_FACTOR
                        + BAND * WORD_SCALE_FACTOR * 1.25,
                ),
                HOLE_SIZE * WORD_SCALE_FACTOR,
            ),
            Col(hole_color),
        );

        self.draw_ball(window, &self.ball_8_handle, &Color::BLACK);
        self.draw_ball(window, &self.white_ball_handle, &Color::WHITE);

        for ball_handle in self.red_balls_handles.iter() {
            self.draw_ball(window, ball_handle, &Color::RED);
        }
        for ball_handle in self.yellow_balls_handles.iter() {
            self.draw_ball(window, ball_handle, &Color::YELLOW);
        }

        let cane_len = 900.;
        if !self.has_force() {
            let queue = Cuboid::new(Vector2::new(cane_len * WORD_SCALE_FACTOR, 2.));
            let ball_object = self.world.body_part(self.white_ball_handle);
            let pos = ball_object.position().clone();
            let mut pos = pos.translation.vector;

            let rot = self.cane_rotation.to_radians();
            pos.x = pos.x - (cane_len + BALL_SIZE + (self.cane_force)) * rot.cos();
            pos.y = pos.y - (cane_len + BALL_SIZE + (self.cane_force)) * rot.sin();
            window.draw_ex(
                &Rectangle::from_cuboid(FromNPVec(pos), &queue),
                Col(Color::RED),
                Transform::rotate(self.cane_rotation),
                0, // we don't really care about the Z value
            );
        }

        Ok(())
    }

    fn update(&mut self, window: &mut Window) -> Result<()> {
        self.world.step();
        for contact in self.world.contact_events() {
            // Handle contact events.
            self.handle_contact_event(contact)
        }

        if self.has_force() {
            self.speed_up_inactive_balls();
            return Ok(());
        }

        if window.keyboard()[Key::Right].is_down() {
            if window.keyboard()[Key::LControl].is_down() {
                self.cane_rotation += 15.;
            } else {
                self.cane_rotation += 0.5;
            }
        }
        if window.keyboard()[Key::Left].is_down() {
            if window.keyboard()[Key::LControl].is_down() {
                self.cane_rotation -= 15.;
            } else {
                self.cane_rotation -= 0.5;
            }
        }

        if window.keyboard()[Key::Down].is_down() {
            self.cane_force += 50.;
        }
        if window.keyboard()[Key::Up].is_down() {
            self.cane_force -= 50.;
        }
        if self.cane_force < 0. {
            self.cane_force = 0.;
        }
        if self.cane_force > 1350. {
            self.cane_force = 1350.;
        }

        if window.keyboard()[Key::Return].is_down() {
            let rot = self.cane_rotation.to_radians();
            let force = self.cane_force.powf(1.5);
            let can_force_x = force * rot.cos();
            let can_force_y = force * rot.sin();

            self.cane_force = 5.0;
            let ball_object = self.world.rigid_body_mut(self.white_ball_handle).unwrap();
            let vel = Velocity::linear(can_force_x, can_force_y);
            ball_object.set_velocity(vel);
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

    fn speed_up_inactive_ball(&mut self, handle: BodyHandle) {
        let ball_object = self.world.rigid_body_mut(handle.clone()).unwrap();
        let vel = ball_object.velocity();
        if vel.linear.x == 0.0 && vel.linear.y == 0.0 {
            return
        }

        if vel.linear.x.abs() < 100. && vel.linear.y.abs() < 100. {
            ball_object.set_velocity(Velocity::linear(0.0, 0.0));
        }
    }

    fn is_active(&self, handle: &BodyHandle) -> bool {
        let ball_object = self.world.rigid_body(handle.clone()).unwrap();
        return ball_object.is_active();
    }

    fn speed_up_inactive_balls(&mut self) {
        self.speed_up_inactive_ball(self.white_ball_handle);
        self.speed_up_inactive_ball(self.ball_8_handle);

        let balls_handles: Vec<BodyHandle> = self.red_balls_handles.iter().map(|x| x.clone()).collect();
        for ball_handle in balls_handles {
            self.speed_up_inactive_ball(ball_handle);
        }
        let balls_handles: Vec<BodyHandle> = self.yellow_balls_handles.iter().map(|x| x.clone()).collect();
        for ball_handle in balls_handles {
            self.speed_up_inactive_ball(ball_handle);
        }
    }

    fn has_force(&self) -> bool {

        if self.is_active(&self.white_ball_handle) {
            info!("white ball is active");
            return true;
        }

        if self.is_active(&self.ball_8_handle) {
            return true;
        }

        for ball_handle in self.red_balls_handles.iter() {
            if self.is_active(ball_handle) {
                return true;
            }
        }
        for ball_handle in self.yellow_balls_handles.iter() {
            if self.is_active(ball_handle) {
                return true;
            }
        }

        false
    }

    fn draw_ball(&self, window: &mut Window, handle: &BodyHandle, color: &Color) {
        //self.white_ball.draw(window);
        let ball_object = self.world.body_part(handle.clone());
        let pos = ball_object.position().clone();
        let pos = pos.translation.vector;
        //info!("Ball pos: {:?}", pos);
        let ball_ball = Ball::new(BALL_SIZE * WORD_SCALE_FACTOR);

        window.draw(
            &Circle::from_ball(FromNPVec(pos), ball_ball),
            Col(color.clone()),
        );
    }
}

fn main() {
    web_logger::init();
    info!("Starting the pool");
    run::<PoolTable>("PoolTable", Vector::new(1024, 768), Settings::default());
    info!("Started");
}
