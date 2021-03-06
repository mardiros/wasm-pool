#[macro_use]
extern crate log;

use std::vec::Vec;

use nalgebra as na;

use ncollide2d::{
    events::ContactEvent,
    math::{Isometry as Isometry2, Vector as Vector2},
    shape::{Ball, Cuboid, ShapeHandle},
    world::CollisionObjectHandle,
};

use nphysics2d::{
    math::Velocity,
    object::{BodyHandle, BodyStatus, Material},
    solver::SignoriniModel,
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

const DISPLAY_BOUND: bool = false;
const COLLIDER_MARGIN: f32 = 0.1;
const BALL_SIZE: f32 = 240.0;
const WIDTH: f32 = 15440.0;
const HEIGHT: f32 = 8000.;
const MARGIN_TOP: f32 = 3000.;
const MARGIN_LEFT: f32 = 2000.;
const BORDER: f32 = 560.;
const BAND: f32 = 160.;
const HOLE_SIZE: f32 = 320.;

const BALL_RESTITUTION: f32 = 0.4;

const CANE_SIZE: f32 = 1800.;
const HELP_LINE_SIZE: f32 = 2600.;
const HELP_LINE_WIDTH: f32 = 0.15;
const FORCE_STEP: f32 = 50.;
const ANGLE_STEP: f32 = 0.5;
const MAX_FORCE: f32 = 1400.;


const WORD_SCALE_FACTOR: f32 = 0.05;
const TIME_STEP: f32 = 1. / 60.;

const Z_GRAVITY: f32 = -0.86;


pub struct ZGravity {}

impl ZGravity {
    // Creates a new radial force generator.
    pub fn new() -> Self {
        ZGravity {}
    }

    pub fn apply_force(&mut self, world: &mut World<f32>, body: BodyHandle) {
        let mut part = world.body_part_mut(body);
        let mut vel = part.as_ref().velocity();

        vel.linear.x = vel.linear.x * Z_GRAVITY;
        vel.linear.y = vel.linear.y * Z_GRAVITY;

        let inertia = part.as_ref().inertia();
        let force = inertia * vel;
        part.apply_force(&force);
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
    z_gravity: ZGravity,

    holes: Vec<BodyHandle>,
    bounds: Vec<(BodyHandle, CollisionObjectHandle)>,

    white_ball_handle: Option<BodyHandle>,
    white_ball_handle_dropped: Option<BodyHandle>,
    ball_8_handle: Option<BodyHandle>,

    dropped_balls_handles: Vec<BodyHandle>,

    yellow_balls_handles: Vec<BodyHandle>,
    red_balls_handles: Vec<BodyHandle>,
}

impl PoolTable {
    fn new() -> PoolTable {
        let mut world: World<f32> = World::new();
        let param = world.integration_parameters_mut();
        param.dt = TIME_STEP;

        let model: SignoriniModel<f32> = SignoriniModel::new();
        world.set_contact_model(model);

        let z_gravity = ZGravity::new();

        PoolTable {
            world,
            z_gravity,
            holes: Vec::with_capacity(6),
            bounds: Vec::new(),
            white_ball_handle: None,
            white_ball_handle_dropped: None,
            ball_8_handle: None,
            dropped_balls_handles: Vec::new(),

            yellow_balls_handles: Vec::new(),
            red_balls_handles: Vec::new(),
        }
    }
    fn initialize_bounds(&mut self) {
        let vertical_height = HEIGHT + 2. * BAND;
        let vertical_height_thin = HEIGHT - HOLE_SIZE - 3. * BAND;
        let horizontal_width = WIDTH + 2. * BAND;
        let horizontal_width_thin = horizontal_width - HOLE_SIZE - 2. * BAND;
        let half_horizontal_width_thin = (horizontal_width - 3. * HOLE_SIZE - 7. * BAND) / 2.;

        let top = MARGIN_TOP + BORDER;
        let left = MARGIN_LEFT + BORDER;

        let half_band = BAND / 2.;
        let half_vertical_height = vertical_height / 2.;
        let half_horizontal_width = horizontal_width / 2.;
        let half_half_horizontal_width_thin = half_horizontal_width_thin / 2.;
        let half_vertical_height_thin = vertical_height_thin / 2.;

        // top
        self.add_bound(
            left + half_horizontal_width,
            top - BAND,
            half_horizontal_width,
            BAND,
        );
        self.add_bound(
            left + HOLE_SIZE + BAND * 1.5 + half_half_horizontal_width_thin,
            top + half_band,
            half_half_horizontal_width_thin,
            half_band,
        );

        self.add_bound(
            left + 2. * HOLE_SIZE
                + 5.5 * BAND
                + half_horizontal_width_thin
                + half_half_horizontal_width_thin,
            top + half_band,
            half_half_horizontal_width_thin,
            half_band,
        );

        // left
        self.add_bound(
            left - BAND,
            top + half_vertical_height,
            BAND,
            half_vertical_height,
        );
        self.add_bound(
            left + half_band,
            top + HOLE_SIZE + BAND * 1.5 + half_vertical_height_thin,
            half_band,
            half_vertical_height_thin,
        );

        // bottom
        self.add_bound(
            left + half_horizontal_width,
            top + vertical_height + BAND,
            half_horizontal_width,
            BAND,
        );

        self.add_bound(
            left + HOLE_SIZE + BAND * 1.5 + half_half_horizontal_width_thin,
            top + vertical_height_thin + 2. * HOLE_SIZE + 2.5 * BAND,
            half_half_horizontal_width_thin,
            half_band,
        );

        self.add_bound(
            left + 2. * HOLE_SIZE
                + 5.5 * BAND
                + half_horizontal_width_thin
                + half_half_horizontal_width_thin,
            top + vertical_height_thin + 2. * HOLE_SIZE + 2.5 * BAND,
            half_half_horizontal_width_thin,
            half_band,
        );

        // right
        self.add_bound(
            left + horizontal_width + BAND,
            top + half_vertical_height,
            BAND,
            half_vertical_height,
        );

        self.add_bound(
            left + horizontal_width_thin + HOLE_SIZE + BAND * 1.5,
            top + HOLE_SIZE + BAND * 1.5 + half_vertical_height_thin,
            half_band,
            half_vertical_height_thin,
        );
    }

    fn initialize_balls(&mut self) {
        let center_y = MARGIN_TOP + BORDER + HEIGHT * 0.5;
        let white_ball_handle = self.add_ball(MARGIN_LEFT + BORDER + WIDTH * 0.25, center_y);

        let center_x = MARGIN_LEFT + BORDER + WIDTH * 0.75;

        //     r
        let ball_r1 = self.add_ball(center_x - 4. * BALL_SIZE, center_y);

        //    y r  ( right to left )

        let ball_r2 = self.add_ball(center_x - 2. * BALL_SIZE, center_y - 1. * BALL_SIZE);

        let ball_y1 = self.add_ball(center_x - 2. * BALL_SIZE, center_y + 1. * BALL_SIZE);

        //   r b y  ( right to left )

        let ball_y2 = self.add_ball(center_x, center_y - 2. * BALL_SIZE);

        let ball_8_handle = self.add_ball(center_x, center_y);

        let ball_r3 = self.add_ball(center_x, center_y + 2. * BALL_SIZE);

        //  y r y r  ( right to left )
        let ball_r4 = self.add_ball(center_x + 2. * BALL_SIZE, center_y - 3. * BALL_SIZE);
        let ball_y3 = self.add_ball(center_x + 2. * BALL_SIZE, center_y - 1. * BALL_SIZE);
        let ball_r5 = self.add_ball(center_x + 2. * BALL_SIZE, center_y + 1. * BALL_SIZE);
        let ball_y4 = self.add_ball(center_x + 2. * BALL_SIZE, center_y + 3. * BALL_SIZE);

        // r y r y y ( right to left )
        let ball_y5 = self.add_ball(center_x + 4. * BALL_SIZE, center_y - 4. * BALL_SIZE);
        let ball_y6 = self.add_ball(center_x + 4. * BALL_SIZE, center_y - 2. * BALL_SIZE);
        let ball_r6 = self.add_ball(center_x + 4. * BALL_SIZE, center_y);
        let ball_y7 = self.add_ball(center_x + 4. * BALL_SIZE, center_y + 2. * BALL_SIZE);
        let ball_r7 = self.add_ball(center_x + 4. * BALL_SIZE, center_y + 4. * BALL_SIZE);

        self.red_balls_handles = vec![
            ball_r1, ball_r2, ball_r3, ball_r4, ball_r5, ball_r6, ball_r7,
        ];
        self.yellow_balls_handles = vec![
            ball_y1, ball_y2, ball_y3, ball_y4, ball_y5, ball_y6, ball_y7,
        ];

        self.ball_8_handle = Some(ball_8_handle);
        self.white_ball_handle = Some(white_ball_handle);
    }

    fn initialize_holes(&mut self) {
        // add hole sensors
        //

        // top left
        self.add_hole(
            MARGIN_LEFT + BORDER + BAND * 0.5,
            MARGIN_TOP + BORDER + BAND * 0.5,
        );
        // top
        self.add_hole(
            MARGIN_LEFT + BORDER + BAND + WIDTH * 0.5,
            MARGIN_TOP + BORDER + BAND * 0.5,
        );

        // top right
        self.add_hole(
            MARGIN_LEFT + BORDER + BAND + WIDTH + BAND * 0.5,
            MARGIN_TOP + BORDER + BAND * 0.5,
        );

        // bottom right hole
        self.add_hole(
            MARGIN_LEFT + BORDER + BAND + WIDTH + BAND * 0.25,
            MARGIN_TOP + BORDER + HEIGHT + BAND * 1.25,
        );
        // bottom hole
        self.add_hole(
            MARGIN_LEFT + BORDER + BAND + WIDTH * 0.5,
            MARGIN_TOP + BORDER + HEIGHT + BAND * 1.5,
        );

        // bottom left hole
        self.add_hole(
            MARGIN_LEFT + BORDER + BAND * 0.5,
            MARGIN_TOP + BORDER + HEIGHT + BAND * 1.5,
        );
    }

    fn initialze_world(&mut self) {
        self.initialize_holes();
        self.initialize_bounds();
        self.initialize_balls();

        //let mut z_gravity: ZGravity = ZGravity::new(Vec::new());
        //self.world.add_force_generator(z_gravity);
    }

    fn add_ball(&mut self, x: f32, y: f32) -> BodyHandle {
        let ball_shape = self.ball_shape();
        let ball_material = self.ball_material();
        let ball_pos = Isometry2::new(Vector2::new(x, y), na::zero());

        let inertia = ball_shape.inertia(1.);
        let center_of_mass = ball_shape.center_of_mass();
        let ball_handle = self.world.add_rigid_body(ball_pos, inertia, center_of_mass);
        // z_gravity.add_body_part(ball_handle);

        self.world.add_collider(
            COLLIDER_MARGIN,
            ball_shape.clone(),
            ball_handle,
            Isometry2::identity(),
            ball_material.clone(),
        );
        ball_handle
    }

    pub fn add_hole(&mut self, x: f32, y: f32) {
        // the hole size does not collide on the displayed border, the ball must enter in it.
        // we fake the display right now.
        let hole_shape: ShapeHandle<f32> = ShapeHandle::new(Ball::new(HOLE_SIZE - BALL_SIZE * 0.5));
        let inertia = hole_shape.inertia(1.0);
        let center_of_mass = hole_shape.center_of_mass();

        let pos = Vector2::new(x, y);
        let pos = Isometry2::new(pos, na::zero());
        let hole = self.world.add_rigid_body(pos, inertia, center_of_mass);
        self.world
            .rigid_body_mut(hole)
            .unwrap()
            .set_status(BodyStatus::Static);
        self.world.add_collider(
            COLLIDER_MARGIN,
            hole_shape,
            hole,
            Isometry2::identity(),
            Material::default(),
        );
        self.holes.push(hole);
    }

    pub fn add_bound(&mut self, center_x: f32, center_y: f32, width: f32, height: f32) {
        let bound_shape: ShapeHandle<f32> =
            ShapeHandle::new(Cuboid::new(Vector2::new(width, height)));
        let inertia = bound_shape.inertia(1.0);
        let center_of_mass = bound_shape.center_of_mass();

        let pos = Isometry2::new(Vector2::new(center_x, center_y), na::zero());
        let bound = self.world.add_rigid_body(pos, inertia, center_of_mass);
        self.world
            .rigid_body_mut(bound)
            .unwrap()
            .set_status(BodyStatus::Static);
        let collider = self.world.add_collider(
            COLLIDER_MARGIN,
            bound_shape,
            BodyHandle::ground(),
            pos,
            self.bound_material(),
        );
        self.bounds.push((bound, collider));
    }

    fn ball_material(&self) -> Material<f32> {
        Material::new(BALL_RESTITUTION, 0.)
    }

    fn ball_shape(&self) -> ShapeHandle<f32> {
        ShapeHandle::new(Ball::new(BALL_SIZE))
    }

    fn bound_material(&self) -> Material<f32> {
        Material::new(0.95, 0.)
    }

    fn drop_ball(&mut self, ball: &BodyHandle) {
        if self.dropped_balls_handles.contains(ball) {
            info!("!!! ball dropped");
            return;
        }

        if Some(*ball) == self.white_ball_handle {
            info!("!!! drop the white ball");
            self.white_ball_handle_dropped = self.white_ball_handle;
            self.white_ball_handle = None;
        } else if Some(*ball) == self.ball_8_handle {
            info!("!!! drop the 8 ball",);
            self.ball_8_handle = None;
        } else if self.red_balls_handles.contains(ball) {
            info!("!!! drop a red ball",);
            self.red_balls_handles = self
                .red_balls_handles
                .iter()
                .filter(|b| b != &ball)
                .map(|b| b.clone())
                .collect();
        } else if self.yellow_balls_handles.contains(ball) {
            info!("!!! drop a yellow ball");
            self.yellow_balls_handles = self
                .yellow_balls_handles
                .iter()
                .filter(|b| b != &ball)
                .map(|b| b.clone())
                .collect();
        }
        self.dropped_balls_handles.push(ball.clone());
        self.world.remove_bodies(&[ball.clone()]);
    }

    fn handle_contact_event(&self, event: &ContactEvent) -> Option<BodyHandle> {
        if let &ContactEvent::Started(collider1, collider2) = event {
            let body1 = self.world.collider(collider1).unwrap().data().body();
            let body2 = self.world.collider(collider2).unwrap().data().body();

            if self.holes.contains(&body1) {
                return Some(body2);
            }
            if self.holes.contains(&body2) {
                return Some(body1);
            }
        }
        None
    }

    fn respawn_white_ball(&mut self) {
        let x = MARGIN_LEFT + BORDER + WIDTH * 0.25;
        let y = MARGIN_TOP + BORDER + HEIGHT * 0.5;

        // XXX inneficient
        let ball = self.add_ball(x, y);
        self.dropped_balls_handles = self
            .dropped_balls_handles
            .iter()
            .filter(|b| **b != ball)
            .map(|b| b.clone())
            .collect();
        self.white_ball_handle = Some(ball);

        self.white_ball_handle_dropped = None;
    }

    fn speed_up_inactive_ball(&mut self, handle: BodyHandle) {
        let ball_object = self.world.rigid_body_mut(handle.clone()).unwrap();
        let vel = ball_object.velocity();
        if vel.linear.x == 0.0 && vel.linear.y == 0.0 {
            return;
        }

        if vel.linear.x.abs() < 150. && vel.linear.y.abs() < 150. {
            ball_object.set_velocity(Velocity::linear(0.0, 0.0));
        }
    }

    fn is_active(&self, handle: Option<BodyHandle>) -> bool {
        if let Some(ball) = handle {
            let ball_object = self.world.rigid_body(ball).unwrap();
            return ball_object.is_active();
        };
        return false;
    }

    fn speed_up_inactive_balls(&mut self) {
        if let Some(ball) = self.white_ball_handle {
            self.speed_up_inactive_ball(ball);
        }
        if let Some(ball) = self.ball_8_handle {
            self.speed_up_inactive_ball(ball);
        }

        let balls_handles: Vec<BodyHandle> =
            self.red_balls_handles.iter().map(|x| x.clone()).collect();
        for ball_handle in balls_handles {
            self.speed_up_inactive_ball(ball_handle);
        }
        let balls_handles: Vec<BodyHandle> = self
            .yellow_balls_handles
            .iter()
            .map(|x| x.clone())
            .collect();
        for ball_handle in balls_handles {
            self.speed_up_inactive_ball(ball_handle);
        }
    }

    fn has_force(&self) -> bool {
        if self.is_active(self.white_ball_handle) {
            return true
        }

        if self.is_active(self.ball_8_handle) {
            return true
        }

        for ball_handle in self.red_balls_handles.iter() {
            if self.is_active(Some(*ball_handle)) {
                return true
            }
        }
        for ball_handle in self.yellow_balls_handles.iter() {
            if self.is_active(Some(*ball_handle)) {
                return true
            }
        }

        false
    }

    fn shoot(&mut self, cane_force_x: f32, cane_force_y: f32) {
        info!("Apply force {} {}", cane_force_x, cane_force_y);
        let ball_object = self
            .world
            .rigid_body_mut(self.white_ball_handle.unwrap())
            .unwrap();
        let vel = Velocity::linear(cane_force_x, cane_force_y);
        ball_object.set_velocity(vel);
    }

    fn in_world(&self, handle: BodyHandle) -> bool {
        let ball_object = self
            .world
            .body_part(handle);
        let pos = ball_object.position().clone();
        let pos = pos.translation.vector;
        pos.x > 0. && pos.x < (WIDTH + 2.*BORDER + MARGIN_LEFT) && pos.y > 0. && pos.y < (HEIGHT + 2.*BORDER + MARGIN_TOP)
    }

    fn step(&mut self) {
        self.world.step();

        // Apply the Zgravity manually
        if let Some(ball) = self.white_ball_handle {
            if self.in_world(ball) {
                self.z_gravity.apply_force(&mut self.world, ball);
            }
            else {
                self.drop_ball(&ball);
            }
        }
        if let Some(ball) = self.ball_8_handle {
            self.z_gravity.apply_force(&mut self.world, ball);
        }
        for ball in self.yellow_balls_handles.iter() {
            self.z_gravity.apply_force(&mut self.world, ball.clone());
        }
        for ball in self.red_balls_handles.iter() {
            self.z_gravity.apply_force(&mut self.world, ball.clone());
        }

        let mut balls = vec![];
        for contact in self.world.contact_events() {
            // Handle contact events.
            if let Some(ball) = self.handle_contact_event(contact) {
                balls.push(ball);
            }
        }
        for ball in balls {
            self.drop_ball(&ball);
        }

        if self.has_force() {
            self.speed_up_inactive_balls();
        }
    }
}

struct PoolGameUI {
    pool_table: PoolTable,
    cane_rotation: f32,
    cane_force: f32,
}

impl State for PoolGameUI {
    fn new() -> Result<PoolGameUI> {
        let mut pool_table = PoolTable::new();
        pool_table.initialze_world();

        let cane_rotation = 0.;
        let cane_force = 5.;

        Ok(PoolGameUI {
            pool_table,
            cane_rotation,
            cane_force,
        })
    }

    fn draw(&mut self, window: &mut Window) -> Result<()> {
        self.draw_table(window)?;

        for hole in self.pool_table.holes.iter() {
            self.draw_hole(window, hole);
        }

        if DISPLAY_BOUND {
            for (bound, collision_object) in self.pool_table.bounds.iter() {
                self.draw_bound(window, bound, collision_object);
            }
        }

        if let Some(ball) = self.pool_table.ball_8_handle {
            self.draw_ball(window, &ball, &Color::BLACK);
        }
        if let Some(ball) = self.pool_table.white_ball_handle {
            self.draw_ball(window, &ball, &Color::WHITE);
        }

        for ball_handle in self.pool_table.red_balls_handles.iter() {
            self.draw_ball(window, ball_handle, &Color::RED);
        }
        for ball_handle in self.pool_table.yellow_balls_handles.iter() {
            self.draw_ball(window, ball_handle, &Color::YELLOW);
        }

        if !self.pool_table.has_force() {
            let queue = Cuboid::new(Vector2::new(CANE_SIZE * WORD_SCALE_FACTOR, 2.));
            if self.pool_table.white_ball_handle.is_none() {
                self.pool_table.respawn_white_ball();
            }
            let ball_object = self
                .pool_table
                .world
                .body_part(self.pool_table.white_ball_handle.unwrap());
            let pos = ball_object.position().clone();
            let mut pos = pos.translation.vector;

            let rot = self.cane_rotation.to_radians();
            pos.x = pos.x - (CANE_SIZE + BALL_SIZE + (self.cane_force * 2.5)) * rot.cos();
            pos.y = pos.y - (CANE_SIZE + BALL_SIZE + (self.cane_force * 2.5)) * rot.sin();
            window.draw_ex(
                &Rectangle::from_cuboid(FromNPVec(pos), &queue),
                Col(Color::RED),
                Transform::rotate(self.cane_rotation),
                0, // we don't really care about the Z value
            );

            let queue = Cuboid::new(Vector2::new(HELP_LINE_SIZE * WORD_SCALE_FACTOR, HELP_LINE_WIDTH));
            let pos = ball_object.position().clone();
            let mut pos = pos.translation.vector;

            let rot = self.cane_rotation.to_radians();
            pos.x = pos.x + (HELP_LINE_SIZE + BALL_SIZE + (self.cane_force * 5.)) * rot.cos();
            pos.y = pos.y + (HELP_LINE_SIZE + BALL_SIZE + (self.cane_force * 5.)) * rot.sin();
            window.draw_ex(
                &Rectangle::from_cuboid(FromNPVec(pos), &queue),
                Col(Color::BLUE),
                Transform::rotate(self.cane_rotation),
                0, // we don't really care about the Z value
            );
        }

        Ok(())
    }

    fn update(&mut self, window: &mut Window) -> Result<()> {
        self.pool_table.step();

        let mut force = FORCE_STEP;
        let mut angle = ANGLE_STEP;

        if window.keyboard()[Key::LControl].is_down()
            || window.keyboard()[Key::RControl].is_down()
        {
            force = force * 10.;
            angle = angle * 10.;
        } else if window.keyboard()[Key::LAlt].is_down() {
            force = force * 0.2;
            angle = angle * 0.2;
        }

        if window.keyboard()[Key::Right].is_down() {
            self.cane_rotation += angle;
        }
        if window.keyboard()[Key::Left].is_down() {
            self.cane_rotation -= angle;
        }
        if window.keyboard()[Key::Down].is_down() {
            self.cane_force += force;
        }
        if window.keyboard()[Key::Up].is_down() {
            self.cane_force -= force;
        }
        if self.cane_force < 0. {
            self.cane_force = 0.;
        }
        if self.cane_force > MAX_FORCE {
            self.cane_force = MAX_FORCE;
        }

        if window.keyboard()[Key::Return].is_down() {
            let rot = self.cane_rotation.to_radians();
            let force = self.cane_force.powf(1.5);
            let cane_force_x = force * rot.cos();
            let cane_force_y = force * rot.sin();

            if !self.pool_table.has_force() {
                self.pool_table.shoot(cane_force_x, cane_force_y);
                self.cane_force = FORCE_STEP;
            }
        }

        Ok(())
    }
}

impl PoolGameUI {
    fn draw_ball(&self, window: &mut Window, handle: &BodyHandle, color: &Color) {
        //self.white_ball.draw(window);
        let ball_object = self.pool_table.world.body_part(handle.clone());
        let pos = ball_object.position().clone();
        let pos = pos.translation.vector;
        //info!("Ball pos: {:?}", pos);
        let ball_ball = Ball::new(BALL_SIZE * WORD_SCALE_FACTOR);

        window.draw(
            &Circle::from_ball(FromNPVec(pos), ball_ball),
            Col(color.clone()),
        );
    }

    fn hole_color(&self) -> Color {
        Color::WHITE
            .with_red(0x22 as f32 / 0xff as f32)
            .with_green(0x22 as f32 / 0xff as f32)
            .with_blue(0x22 as f32 / 0xff as f32)
    }

    fn draw_bound(
        &self,
        window: &mut Window,
        handle: &BodyHandle,
        collision_object: &CollisionObjectHandle,
    ) {
        let bound_object = self.pool_table.world.body_part(handle.clone());
        let shape: &Cuboid<f32> = self
            .pool_table
            .world
            .collision_world()
            .collision_object(collision_object.clone())
            .unwrap()
            .shape()
            .as_shape()
            .unwrap();
        let pos = bound_object.position().clone();
        let pos = pos.translation.vector;

        let half_size: Vector2<f32> = shape.half_extents().clone().into();
        let vecr = Vector::new(
            (pos.x - half_size.x) * WORD_SCALE_FACTOR,
            (pos.y - half_size.y) * WORD_SCALE_FACTOR,
        );
        let rect = Rectangle::new(vecr, half_size * 2. * WORD_SCALE_FACTOR);

        window.draw(&rect, Col(Color::RED));
    }

    fn draw_hole(&self, window: &mut Window, handle: &BodyHandle) {
        let hole_object = self.pool_table.world.body_part(handle.clone());
        let pos = hole_object.position().clone();
        let pos = pos.translation.vector;
        //info!("Ball pos: {:?}", pos);
        let ball_ball = Ball::new(HOLE_SIZE * WORD_SCALE_FACTOR);

        window.draw(
            &Circle::from_ball(FromNPVec(pos), ball_ball),
            Col(self.hole_color()),
        );
    }

    fn draw_table(&self, window: &mut Window) -> Result<()> {
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

        window.draw(
            &Rectangle::new(
                (
                    MARGIN_LEFT * WORD_SCALE_FACTOR,
                    MARGIN_TOP * WORD_SCALE_FACTOR,
                ),
                (
                    (WIDTH + BORDER * 2. + BAND * 2.) * WORD_SCALE_FACTOR,
                    (HEIGHT + BORDER * 2. + BAND * 2.) * WORD_SCALE_FACTOR,
                ),
            ),
            Col(border_color),
        );
        window.draw(
            &Rectangle::new(
                (
                    (MARGIN_LEFT + BORDER) * WORD_SCALE_FACTOR,
                    (MARGIN_TOP + BORDER) * WORD_SCALE_FACTOR,
                ),
                (
                    (WIDTH + BAND * 2.) * WORD_SCALE_FACTOR,
                    (HEIGHT + BAND * 2.) * WORD_SCALE_FACTOR,
                ),
            ),
            Col(band_color),
        );
        window.draw(
            &Rectangle::new(
                (
                    (MARGIN_LEFT + BORDER + BAND) * WORD_SCALE_FACTOR,
                    (MARGIN_TOP + BORDER + BAND) * WORD_SCALE_FACTOR,
                ),
                (WIDTH * WORD_SCALE_FACTOR, HEIGHT * WORD_SCALE_FACTOR),
            ),
            Col(table_color),
        );

        Ok(())
    }
}

fn main() {
    web_logger::init();
    info!("Starting the pool");
    run::<PoolGameUI>("PoolTable", Vector::new(1024, 768), Settings::default());
    info!("Started");
}
