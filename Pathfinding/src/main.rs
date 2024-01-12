// Potential future optimizations:
// Cache line-of-sight checks
// Don't use line-of-sight for neighbor queries

use std::{collections::HashMap, error::Error, time::Instant};

use bytes::{Buf, BufMut, Bytes, BytesMut};
use collision::{Circle, CollisionShape, Rectangle};
use messenger_client::{Message, MessengerClient};
use tokio::fs;
use uuid::Uuid;
use vectors::{Vec2f, Vec2i};

pub mod collision;
pub mod config;
pub mod dijkstra;
pub mod grid;
pub mod theta_star;
pub mod vectors;

const CONFIG_FILE_NAME: &str = "config.json";

const MSG_SET_ENDPOINTS: &str = "Pathfinder:SetEndpoints";
const MSG_PATH: &str = "Pathfinder:Path";

const MSG_GET_FIELD_INFO: &str = "Pathfinder:GetFieldInfo";
const MSG_GET_CELL_DATA: &str = "Pathfinder:GetCellData";
const MSG_GET_SHAPES: &str = "Pathfinder:GetShapes";

const MSG_SET_SHAPE: &str = "Pathfinder:SetShape";
const MSG_REMOVE_SHAPE: &str = "Pathfinder:RemoveShape";
const MSG_SET_DYN_SHAPES: &str = "Pathfinder:SetDynamicShapes";

const MSG_FIELD_INFO: &str = "Pathfinder:FieldInfo";
const MSG_CELL_DATA: &str = "Pathfinder:CellData";
const MSG_SHAPES: &str = "Pathfinder:Shapes";

// TODO: This should almost certainly be split up at some point
struct PathfinderTask {
    messenger: MessengerClient,
    static_shapes: HashMap<Uuid, (String, CollisionShape)>,
    dyn_shapes: Vec<CollisionShape>,
    robot_radius: f64,
    dyn_grid: grid::Grid2D,
    static_grid: grid::Grid2D,

    cell_size: f64,
    origin_pos_cell: Vec2f,
    field_size: Vec2f,
    env_file_name: String,
}

impl PathfinderTask {
    async fn new() -> Result<Self, Box<dyn Error>> {
        let conf: config::Config = config::load_or_save_default(CONFIG_FILE_NAME).await?;
        let env: config::Environment = config::load_or_save_default(&conf.env_file).await?;

        let msg_addr = format!("{}:{}", conf.messenger.host, conf.messenger.port);
        let mut messenger = MessengerClient::new(msg_addr, conf.messenger.name);
        messenger.listen_multiple(vec![
            MSG_SET_ENDPOINTS,
            MSG_GET_FIELD_INFO,
            MSG_GET_CELL_DATA,
            MSG_GET_SHAPES,
            MSG_SET_SHAPE,
            MSG_REMOVE_SHAPE,
            MSG_SET_DYN_SHAPES,
        ]);

        let shapes = env.shapes;
        let grid = grid::Grid2D::new(Vec2i {
            x: (env.width / conf.field.cell_size).ceil() as i32,
            y: (env.height / conf.field.cell_size).ceil() as i32,
        });

        let mut task = PathfinderTask {
            messenger,
            static_shapes: shapes,
            dyn_shapes: Vec::new(),
            robot_radius: conf.robot_radius,
            cell_size: conf.field.cell_size,
            origin_pos_cell: Vec2f {
                x: grid.cell_size().x as f64 * conf.field.origin_x,
                y: grid.cell_size().y as f64 * conf.field.origin_y,
            },
            static_grid: grid.clone(),
            dyn_grid: grid,
            field_size: Vec2f {
                x: env.width,
                y: env.height,
            },
            env_file_name: conf.env_file,
        };

        for (_, (_, shape)) in &task.static_shapes.clone() {
            task.update_add_static_shape(shape);
        }

        Ok(task)
    }

    fn cell_to_meters(&self, cell: &Vec2f) -> Vec2f {
        Vec2f {
            x: (cell.x - self.origin_pos_cell.x) * self.cell_size,
            y: -(cell.y - self.origin_pos_cell.y) * self.cell_size,
        }
    }

    fn meters_to_cell(&self, meters: &Vec2f) -> Vec2f {
        Vec2f {
            x: meters.x / self.cell_size + self.origin_pos_cell.x,
            y: self.origin_pos_cell.y - meters.y / self.cell_size,
        }
    }

    fn cell_center(&self, cell: &Vec2i) -> Vec2f {
        self.cell_to_meters(&(Vec2f::from(cell) + Vec2f { x: 0.5, y: 0.5 }))
    }

    async fn save_env_file(&self) -> Result<(), Box<dyn Error>> {
        let new_env = config::Environment {
            width: self.field_size.x,
            height: self.field_size.y,
            shapes: self.static_shapes.clone(),
        };
        let env_str = serde_json::to_string_pretty(&new_env)?;
        fs::write(&self.env_file_name, &env_str).await?;
        println!("Saved environment file");
        Ok(())
    }

    fn get_robot_circle(&self, cell_pos: &Vec2i) -> Circle {
        let robot_pos = self.cell_center(cell_pos);
        Circle {
            position: robot_pos,
            radius: self.robot_radius,
        }
    }

    fn constrain_to_grid(&self, cell: Vec2i) -> Vec2i {
        let sz = self.dyn_grid.cell_size();
        return Vec2i {
            x: cell.x.clamp(0, sz.x),
            y: cell.y.clamp(0, sz.y),
        };
    }

    fn get_shape_region(&self, shape: &CollisionShape) -> (Vec2i, Vec2i) {
        let aabb = shape.get_bounding_box().inflate(self.robot_radius);
        let min = self.meters_to_cell(&aabb.min).floor();
        let max = self.meters_to_cell(&aabb.max).ceil();
        (
            self.constrain_to_grid(Vec2i { x: min.x, y: max.y }),
            self.constrain_to_grid(Vec2i { x: max.x, y: min.y }),
        )
    }

    // When adding static shape:
    //   In shape's bounding box, if currently passable and shape collides, set not passable
    //   Also set not passable in dynamic grid
    fn update_add_static_shape(&mut self, shape: &CollisionShape) {
        let (min, max) = self.get_shape_region(shape);

        for y in min.y..max.y {
            for x in min.x..max.x {
                // Don't bother checking, we already can't go here
                if !self.static_grid.can_cell_pass(x, y) {
                    continue;
                }

                let cell_pos = Vec2i { x, y };
                let robot = self.get_robot_circle(&cell_pos);

                if shape.collides_with_circle(&robot) {
                    self.static_grid.set_cell_passable(&cell_pos, false);
                    self.dyn_grid.set_cell_passable(&cell_pos, false);
                }
            }
        }
    }

    // When removing static shape:
    //   In shape's bounding box, if did collide with removed shape, recalculate with all others
    //   If it got cleared, recalculate dynamic cell as well
    fn update_remove_static_shape(&mut self, shape: &CollisionShape) {
        let (min, max) = self.get_shape_region(shape);

        for y in min.y..max.y {
            'cell_loop: for x in min.x..max.x {
                let cell_pos = Vec2i { x, y };
                let robot = self.get_robot_circle(&cell_pos);

                if shape.collides_with_circle(&robot) {
                    for (_, (_, shape)) in &self.static_shapes {
                        if shape.collides_with_circle(&robot) {
                            continue 'cell_loop;
                        }
                    }
                    self.static_grid.set_cell_passable(&cell_pos, true);

                    for shape in &self.dyn_shapes {
                        if shape.collides_with_circle(&robot) {
                            continue 'cell_loop;
                        }
                    }
                    self.dyn_grid.set_cell_passable(&cell_pos, true);
                }
            }
        }
    }

    // When setting dynamic shapes:
    //   Copy all cell data from static grid into dynamic grid
    //   For each new dynamic shape, do same as adding static shape
    fn update_set_dyn_shapes(&mut self) {
        let start_time = Instant::now();
        // Copy in all the datas
        self.dyn_grid = self.static_grid.clone();

        for shape in &self.dyn_shapes {
            let (min, max) = self.get_shape_region(shape);

            for y in min.y..max.y {
                for x in min.x..max.x {
                    // Don't bother checking, we already can't go here
                    if !self.dyn_grid.can_cell_pass(x, y) {
                        continue;
                    }

                    let cell_pos = Vec2i { x, y };
                    let robot = self.get_robot_circle(&cell_pos);

                    if shape.collides_with_circle(&robot) {
                        self.dyn_grid.set_cell_passable(&cell_pos, false);
                    }
                }
            }
        }
        let end_time = Instant::now();

        println!(
            "Regenerating dynamic grid took {} secs",
            end_time.duration_since(start_time).as_secs_f64(),
        );
    }

    async fn save_static_shapes(&mut self) {
        match self.save_env_file().await {
            Ok(_) => {}
            Err(e) => {
                eprintln!("Failed to save environment: {}", e);
            }
        }
    }

    async fn run(mut self) {
        println!("Pathfinder is running");

        let mut start_pos = Vec2f { x: 0.0, y: 0.0 };
        let mut goal_pos = Vec2f { x: 0.0, y: 0.0 };
        let mut needs_recalc = false;

        loop {
            for msg in self.messenger.poll_or_await_messages().await {
                let mut data = msg.data;
                match msg.name.as_str() {
                    MSG_SET_ENDPOINTS => {
                        let start_x = data.get_f64();
                        let start_y = data.get_f64();
                        let goal_x = data.get_f64();
                        let goal_y = data.get_f64();
                        start_pos = Vec2f {
                            x: start_x,
                            y: start_y,
                        };
                        goal_pos = Vec2f {
                            x: goal_x,
                            y: goal_y,
                        };
                        needs_recalc = true;
                    }
                    MSG_GET_FIELD_INFO => self.on_get_field_info(),
                    MSG_GET_CELL_DATA => self.on_get_cell_data(),
                    MSG_GET_SHAPES => self.on_get_shapes(),
                    MSG_SET_SHAPE => self.on_set_shape(data).await,
                    MSG_REMOVE_SHAPE => self.on_remove_shape(data).await,
                    MSG_SET_DYN_SHAPES => {
                        self.on_set_dyn_shapes(&mut data);

                        let start_x = data.get_f64();
                        let start_y = data.get_f64();
                        start_pos = Vec2f {
                            x: start_x,
                            y: start_y,
                        };
                        needs_recalc = true;
                    }
                    _ => {}
                }
            }

            if needs_recalc {
                let start_time = Instant::now();

                let real_start_cell = self.find_nearest_point(start_pos);
                let start_cell = dijkstra::find_nearest_passable(&self.dyn_grid, real_start_cell)
                    .unwrap_or(real_start_cell);
                let goal_cell = self.find_nearest_point(goal_pos);

                let mut result = theta_star::find_path(&self.dyn_grid, start_cell, goal_cell);
                let end_time = Instant::now();

                let data = match &mut result {
                    Some(path) => {
                        if start_cell != real_start_cell {
                            path.insert(0, real_start_cell);
                        }

                        let mut buf = BytesMut::with_capacity(5 + 16 * path.len());
                        buf.put_u8(1);
                        buf.put_i32(path.len() as i32);
                        for point in path {
                            let pos_meters = self.cell_to_meters(&point.into());
                            buf.put_f64(pos_meters.x);
                            buf.put_f64(pos_meters.y);
                        }
                        buf
                    }
                    None => BytesMut::zeroed(1),
                };

                self.messenger.send_message(Message {
                    name: MSG_PATH.to_string(),
                    data: data.into(),
                });

                println!(
                    "Calculating path from {:?} to {:?} took {} secs; found={}",
                    start_pos,
                    goal_pos,
                    end_time.duration_since(start_time).as_secs_f64(),
                    result.is_some()
                );
                needs_recalc = false;
            }
        }
    }

    fn find_nearest_point(&self, pos_meters: Vec2f) -> Vec2i {
        let cell_f = self.meters_to_cell(&pos_meters);
        let size = self.dyn_grid.cell_size();
        Vec2i {
            x: (cell_f.x.round() as i32).clamp(0, size.x),
            y: (cell_f.y.round() as i32).clamp(0, size.y),
        }
    }

    // ---- Message handlers ----

    fn on_get_field_info(&mut self) {
        let mut data = BytesMut::with_capacity(48);
        data.put_f64(self.cell_size);
        data.put_f64(self.field_size.x);
        data.put_f64(self.field_size.y);
        data.put_f64(self.origin_pos_cell.x);
        data.put_f64(self.origin_pos_cell.y);
        let grid_sz = self.dyn_grid.cell_size();
        data.put_i32(grid_sz.x);
        data.put_i32(grid_sz.y);

        self.messenger.send_message(Message {
            name: MSG_FIELD_INFO.to_string(),
            data: data.into(),
        });
    }

    fn on_get_cell_data(&mut self) {
        let size = self.dyn_grid.cell_size();
        let bit_data = self.dyn_grid.get_as_java_bitset();

        let mut data = BytesMut::with_capacity(bit_data.len() * 8 + 12);
        data.put_i32(size.x);
        data.put_i32(size.y);
        data.put_i32(bit_data.len() as i32);
        for word in bit_data {
            data.put_u64(word);
        }

        self.messenger.send_message(Message {
            name: MSG_CELL_DATA.to_string(),
            data: data.into(),
        });
    }

    fn write_shape(data: &mut BytesMut, shape: &CollisionShape) {
        match shape {
            CollisionShape::Circle(c) => {
                data.reserve(25);
                data.put_u8(0);
                data.put_f64(c.position.x);
                data.put_f64(c.position.y);
                data.put_f64(c.radius);
            }
            CollisionShape::Rectangle(r) => {
                data.reserve(42);
                data.put_u8(1);
                data.put_f64(r.position.x);
                data.put_f64(r.position.y);
                data.put_f64(r.size.x);
                data.put_f64(r.size.y);
                data.put_f64(r.rotation);
                data.put_u8(if r.inverted { 1 } else { 0 });
            }
        }
    }

    fn on_get_shapes(&mut self) {
        let mut data = BytesMut::with_capacity(4);
        data.put_i32(self.static_shapes.len() as i32);
        for (id, (name, shape)) in &self.static_shapes {
            let name_utf = name.as_bytes();

            data.reserve(17 + 2 + name_utf.len());
            data.put_u128(id.as_u128());
            data.put_u16(name_utf.len() as u16);
            data.put(name_utf);
            Self::write_shape(&mut data, shape);
        }

        data.reserve(4);
        data.put_i32(self.dyn_shapes.len() as i32);
        for shape in &self.dyn_shapes {
            Self::write_shape(&mut data, shape);
        }

        self.messenger.send_message(Message {
            name: MSG_SHAPES.to_string(),
            data: data.into(),
        });
    }

    fn read_shape(data: &mut Bytes) -> Option<CollisionShape> {
        match data.get_u8() {
            0 => {
                let x = data.get_f64();
                let y = data.get_f64();
                let rad = data.get_f64();
                Some(CollisionShape::Circle(Circle {
                    position: Vec2f { x, y },
                    radius: rad,
                }))
            }
            1 => {
                let x = data.get_f64();
                let y = data.get_f64();
                let w = data.get_f64();
                let h = data.get_f64();
                let rot = data.get_f64();
                let inv = data.get_u8() != 0;
                Some(CollisionShape::Rectangle(Rectangle {
                    position: Vec2f { x, y },
                    size: Vec2f { x: w, y: h },
                    rotation: rot,
                    inverted: inv,
                }))
            }
            _ => None,
        }
    }

    async fn on_set_shape(&mut self, mut data: Bytes) {
        let id = Uuid::from_u128(data.get_u128());

        let name_len = data.get_u16() as usize;
        let mut name_data = BytesMut::with_capacity(name_len);
        for _ in 0..name_len {
            name_data.put_u8(data.get_u8());
        }
        let name = String::from_utf8_lossy(&name_data).to_string();

        if let Some(shape) = Self::read_shape(&mut data) {
            self.update_add_static_shape(&shape);
            if let Some((_, old_shape)) = self.static_shapes.insert(id, (name, shape)) {
                self.update_remove_static_shape(&old_shape);
            }
            self.save_static_shapes().await;
        }
    }

    async fn on_remove_shape(&mut self, mut data: Bytes) {
        let id = Uuid::from_u128(data.get_u128());
        if let Some((_, removed_shape)) = self.static_shapes.remove(&id) {
            self.update_remove_static_shape(&removed_shape);
            self.save_static_shapes().await;
        }
    }

    fn on_set_dyn_shapes(&mut self, data: &mut Bytes) {
        let count = data.get_i32();
        self.dyn_shapes.clear();
        for _ in 0..count {
            if let Some(shape) = Self::read_shape(data) {
                self.dyn_shapes.push(shape);
            } else {
                self.update_set_dyn_shapes();
                return;
            }
        }
        self.update_set_dyn_shapes();
    }
}

#[tokio::main]
async fn main() {
    match PathfinderTask::new().await {
        Ok(task) => task.run().await,
        Err(e) => {
            eprintln!("Failed to start pathfinding:");
            eprintln!("{}", e);
        }
    }
}
