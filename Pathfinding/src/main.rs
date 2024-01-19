// TODO:
//   CRITICAL: Make it not freeze if the goal is inside obstacle
//   If inside obstacle, find a way out first
//   OPTIMIZATION: Use bidirectional Dijkstra for fast

use std::{
    error::Error,
    sync::{Arc, Mutex},
    time::Instant,
};

use bytes::{Buf, BufMut, BytesMut};
use itertools::Itertools;
use math::Vec2f;
use messenger_client::{Message, MessengerClient};

mod bezier;
mod config;
mod geom;
mod math;
mod obstacle;

mod gui;

const MSG_SET_ENDPOINTS: &str = "Pathfinder:SetEndpoints";
const MSG_PATH: &str = "Pathfinder:Path";

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let conf: config::Config = config::load_or_save_default("config.json")?;
    let environment: config::EnvironmentConfig = config::load_or_save_default(&conf.env_file)?;

    let addr = format!("{}:{}", conf.messenger.host, conf.messenger.port);
    let mut messenger = MessengerClient::new(addr, conf.messenger.name);
    messenger.listen_multiple(vec![MSG_SET_ENDPOINTS]);

    let obstacles = environment
        .obstacles
        .values()
        .map(|(_, o)| o.clone().into_obstacle())
        .collect_vec();

    let field = Arc::new(geom::Field::generate(
        &obstacles,
        conf.robot_radius + conf.tolerance,
    ));

    let mut start = Vec2f::new(1.0, 1.0);
    let mut goal = Vec2f::new(1.2, 1.2);

    let gui_state = Arc::new(Mutex::new(gui::GraphicsState {
        start,
        goal,
        calc_time: 0.0,
        path: None,
    }));
    gui::show_graphics_window(
        Vec2f::new(environment.width, environment.height),
        conf.robot_radius,
        obstacles,
        field.clone(),
        gui_state.clone(),
    );

    loop {
        let mut needs_recalc = false;
        for msg in messenger.poll_or_await_messages().await {
            let mut data = msg.data;
            match msg.name.as_str() {
                MSG_SET_ENDPOINTS => {
                    let start_x = data.get_f64();
                    let start_y = data.get_f64();
                    let goal_x = data.get_f64();
                    let goal_y = data.get_f64();
                    start = Vec2f {
                        x: start_x,
                        y: start_y,
                    };
                    goal = Vec2f {
                        x: goal_x,
                        y: goal_y,
                    };
                    needs_recalc = true;
                }
                _ => {}
            }
        }

        if needs_recalc {
            let calc_start = Instant::now();
            let path_opt = field.find_path(start, goal);
            let calc_end = Instant::now();
            let calc_time = calc_end.duration_since(calc_start).as_secs_f64();

            if let Ok(mut state) = gui_state.lock() {
                state.start = start;
                state.goal = goal;
                state.calc_time = calc_time;
            }

            let has_path = path_opt.is_some();
            let data = match path_opt {
                Some(path) => {
                    let bezier_pts = bezier::to_bezier(&path, start, goal);

                    let mut buf = BytesMut::with_capacity(bezier_pts.len() * 16 + 4 + 1);
                    buf.put_u8(1);
                    buf.put_i32(bezier_pts.len() as i32);
                    for point in &bezier_pts {
                        buf.put_f64(point.x);
                        buf.put_f64(point.y);
                    }

                    if let Ok(mut state) = gui_state.lock() {
                        state.path = Some((path, bezier_pts));
                    }

                    buf
                }
                None => {
                    if let Ok(mut state) = gui_state.lock() {
                        state.path = None;
                    }
                    BytesMut::zeroed(1)
                }
            };

            messenger.send_message(Message {
                name: MSG_PATH.to_string(),
                data: data.into(),
            });

            println!(
                "Calculating path from {:?} to {:?} took {} secs; found={}",
                start, goal, calc_time, has_path
            );
        }
    }
}

// use std::{error::Error, f64::consts::PI, time::Instant};

// use geom::WindingDir;
// use itertools::Itertools;
// use lerp::Lerp;
// use macroquad::prelude::*;
// use math::Vec2f;
// use obstacle::Obstacle;

// fn draw_arc(center: Vec2f, radius: f64, min: f64, max: f64, thickness: f32, color: Color) {
//     let mut prev_angle = min;
//     for i in 1..=31 {
//         let angle = min.lerp(max, (i as f64) / 31.0);

//         let prev_pos = center + Vec2f::new_angle(radius, prev_angle);
//         let pos = center + Vec2f::new_angle(radius, angle);

//         draw_line(
//             prev_pos.x as f32,
//             prev_pos.y as f32,
//             pos.x as f32,
//             pos.y as f32,
//             thickness,
//             color,
//         );

//         prev_angle = angle;
//     }
// }

// fn draw_bezier(p0: Vec2f, p1: Vec2f, p2: Vec2f, p3: Vec2f, thickness: f32, color: Color) {
//     let mut prev_pt = p0;
//     for i in 1..=16 {
//         let t = i as f64 / 16.0;

//         let a = p0.lerp(p1, t);
//         let b = p1.lerp(p2, t);
//         let c = p2.lerp(p3, t);
//         let d = a.lerp(b, t);
//         let e = b.lerp(c, t);
//         let point = d.lerp(e, t);

//         draw_line(
//             prev_pt.x as f32,
//             prev_pt.y as f32,
//             point.x as f32,
//             point.y as f32,
//             thickness,
//             color,
//         );
//         prev_pt = point;
//     }
// }

// #[macroquad::main("Arc Pathfinding")]
// async fn main() -> Result<(), Box<dyn Error>> {
//     let conf: config::Config = config::load_or_save_default("config.json")?;
//     let environment: config::EnvironmentConfig = config::load_or_save_default(&conf.env_file)?;

//     let obstacles = environment
//         .obstacles
//         .values()
//         .map(|(_, o)| o.clone().into_obstacle())
//         .collect_vec();

//     let gen_start = Instant::now();
//     let field = geom::Field::generate(&obstacles, conf.robot_radius + conf.tolerance);
//     println!("Precalc took {} secs", gen_start.elapsed().as_secs_f64());

//     let mut start = Vec2f::new(1.0, 1.0);
//     let mut goal = Vec2f::new(1.2, 1.2);

//     let mut preview_dist = 0.0;
//     let mut prev_frame_time = Instant::now();

//     loop {
//         let frame_time = Instant::now();
//         let elapsed = frame_time.duration_since(prev_frame_time);
//         prev_frame_time = frame_time;

//         clear_background(BLACK);

//         let scale_x = (screen_width() - 50.0) / environment.width as f32;
//         let scale_y = (screen_height() - 50.0) / environment.height as f32;

//         let scale = scale_x.min(scale_y);
//         let pixel = 1.0 / scale;
//         set_camera(&Camera2D {
//             zoom: Vec2 {
//                 x: 2.0 * scale / screen_width(),
//                 y: 2.0 * -scale / screen_height(),
//             },
//             target: Vec2 {
//                 x: environment.width as f32 / 2.0,
//                 y: environment.height as f32 / 2.0,
//             },
//             ..Default::default()
//         });

//         let (mouse_x, mouse_y) = mouse_position();
//         let mouse_pos = Vec2f {
//             x: ((mouse_x - screen_width() / 2.0) / scale) as f64 + environment.width / 2.0,
//             y: ((screen_height() / 2.0 - mouse_y) / scale) as f64 + environment.height / 2.0,
//         };
//         draw_circle(mouse_pos.x as f32, mouse_pos.y as f32, 15.0 * pixel, BLUE);

//         if is_mouse_button_down(MouseButton::Left) {
//             start = mouse_pos;
//         }
//         if is_mouse_button_down(MouseButton::Right) {
//             goal = mouse_pos;
//         }

//         for obstacle in &obstacles {
//             match obstacle {
//                 Obstacle::Circle(circle) => {
//                     draw_circle_lines(
//                         circle.position.x as f32,
//                         circle.position.y as f32,
//                         circle.radius as f32,
//                         2.0 * pixel,
//                         ORANGE,
//                     );
//                 }
//                 Obstacle::Polygon(polygon) => {
//                     let mut prev = polygon.vertices[polygon.vertices.len() - 1];
//                     for vert in &polygon.vertices {
//                         draw_line(
//                             prev.x as f32,
//                             prev.y as f32,
//                             vert.x as f32,
//                             vert.y as f32,
//                             2.0 * pixel,
//                             ORANGE,
//                         );
//                         prev = *vert;
//                     }
//                 }
//             }
//         }

//         for arc in &field.arcs {
//             let (min, max) = if arc.min_angle == arc.max_angle {
//                 (0.0, PI * 2.0)
//             } else if arc.max_angle < arc.min_angle {
//                 (arc.min_angle, arc.max_angle + PI * 2.0)
//             } else {
//                 (arc.min_angle, arc.max_angle)
//             };

//             draw_arc(arc.center, arc.radius, min, max, 2.0 * pixel, RED);
//         }

//         for segment in &field.segments {
//             draw_line(
//                 segment.from.x as f32,
//                 segment.from.y as f32,
//                 segment.to.x as f32,
//                 segment.to.y as f32,
//                 2.0 * pixel,
//                 GRAY,
//             );
//         }

//         for edge_set in &field.visibility {
//             for edge in edge_set {
//                 let seg = &edge.segment;
//                 draw_line(
//                     seg.from.x as f32,
//                     seg.from.y as f32,
//                     seg.to.x as f32,
//                     seg.to.y as f32,
//                     1.0 * pixel,
//                     MAGENTA,
//                 );
//             }
//         }

//         let calc_start = Instant::now();
//         let path_opt = field.find_path(start, goal);
//         // let path_opt: Option<Vec<geom::PathArc>> = None;
//         let calc_end = Instant::now();

//         if let Some(path) = path_opt {
//             if is_key_down(KeyCode::Space) {
//                 let mut p = start;
//                 for arc in &path {
//                     draw_line(
//                         p.x as f32,
//                         p.y as f32,
//                         (arc.center.x + arc.radius * arc.incoming_angle.cos()) as f32,
//                         (arc.center.y + arc.radius * arc.incoming_angle.sin()) as f32,
//                         4.0 * pixel,
//                         GREEN,
//                     );

//                     let mut angle1 = math::wrap_angle(arc.incoming_angle);
//                     let mut angle2 = math::wrap_angle(arc.outgoing_angle);
//                     if arc.direction == WindingDir::Clockwise {
//                         std::mem::swap(&mut angle1, &mut angle2);
//                     }

//                     if angle2 < angle1 {
//                         angle2 += PI * 2.0;
//                     }

//                     draw_arc(arc.center, arc.radius, angle1, angle2, 4.0 * pixel, GREEN);
//                     p = arc.center + Vec2f::new_angle(arc.radius, arc.outgoing_angle);
//                 }
//                 draw_line(
//                     p.x as f32,
//                     p.y as f32,
//                     goal.x as f32,
//                     goal.y as f32,
//                     4.0 * pixel,
//                     GREEN,
//                 );
//             } else {
//                 let bezier_pts = bezier::to_bezier(&path, start, goal);

//                 let mut anchor = bezier_pts[0];
//                 let curve_count = (bezier_pts.len() - 1) / 3;

//                 for i in 0..curve_count {
//                     let cp1 = bezier_pts[i * 3 + 1];
//                     let cp2 = bezier_pts[i * 3 + 2];
//                     let anchor2 = bezier_pts[i * 3 + 3];

//                     draw_bezier(anchor, cp1, cp2, anchor2, 4.0 * pixel, ORANGE);
//                     anchor = anchor2;
//                 }
//             }

//             preview_dist += elapsed.as_secs_f64() * 2.5;

//             let mut preview_pt = start;
//             let mut dist_so_far = 0.0;
//             let mut preview_found = false;
//             for arc in &path {
//                 let in_pt = arc.center + Vec2f::new_angle(arc.radius, arc.incoming_angle);
//                 let out_pt = arc.center + Vec2f::new_angle(arc.radius, arc.outgoing_angle);

//                 let to_in_dist = preview_pt.distance_sq(in_pt).sqrt();
//                 if dist_so_far + to_in_dist > preview_dist {
//                     preview_pt = preview_pt.lerp(in_pt, (preview_dist - dist_so_far) / to_in_dist);
//                     preview_found = true;
//                     break;
//                 }
//                 dist_so_far += to_in_dist;

//                 let angle1 = math::wrap_angle(arc.incoming_angle);
//                 let angle2 = math::wrap_angle(arc.outgoing_angle);
//                 let diff = math::floor_mod(
//                     match arc.direction {
//                         WindingDir::Counterclockwise => angle2 - angle1,
//                         WindingDir::Clockwise => angle1 - angle2,
//                     },
//                     PI * 2.0,
//                 );
//                 let arc_dist = diff * arc.radius;
//                 if dist_so_far + arc_dist > preview_dist {
//                     let angle = angle1.lerp(angle2, (preview_dist - dist_so_far) / arc_dist);
//                     preview_pt = arc.center + Vec2f::new_angle(arc.radius, angle);
//                     preview_found = true;
//                     break;
//                 }

//                 preview_pt = out_pt;
//                 dist_so_far += arc_dist;
//             }

//             if !preview_found {
//                 let to_goal_dist = preview_pt.distance_sq(goal).sqrt();
//                 if dist_so_far + to_goal_dist > preview_dist {
//                     preview_pt = preview_pt.lerp(goal, (preview_dist - dist_so_far) / to_goal_dist);
//                     preview_found = true;
//                 }
//             }

//             if !preview_found {
//                 preview_dist = 0.0;
//             }

//             draw_circle_lines(
//                 preview_pt.x as f32,
//                 preview_pt.y as f32,
//                 conf.robot_radius as f32,
//                 2.0 * pixel,
//                 GREEN,
//             );
//         }

//         let mut closest = None;
//         for (id, arc) in field.arcs.iter().enumerate() {
//             let dist = arc.center.distance_sq(mouse_pos).sqrt();
//             if dist < arc.radius
//                 && match closest {
//                     Some((_, distance)) => dist < distance,
//                     None => true,
//                 }
//             {
//                 closest = Some((id, dist));
//             }
//         }

//         if let Some((id, _)) = closest {
//             // Clockwise
//             for edge in &field.visibility[id << 1] {
//                 draw_line(
//                     edge.segment.from.x as f32,
//                     edge.segment.from.y as f32,
//                     edge.segment.to.x as f32,
//                     edge.segment.to.y as f32,
//                     4.0 * pixel,
//                     BLUE,
//                 );
//             }

//             // Counterclockwise
//             for edge in &field.visibility[(id << 1) + 1] {
//                 draw_line(
//                     edge.segment.from.x as f32,
//                     edge.segment.from.y as f32,
//                     edge.segment.to.x as f32,
//                     edge.segment.to.y as f32,
//                     4.0 * pixel,
//                     GOLD,
//                 );
//             }
//         }

//         set_default_camera();
//         let calc_time_ms = calc_end.duration_since(calc_start).as_secs_f64() * 1000.0;
//         draw_text(
//             format!("Calc time: {:.3} ms", calc_time_ms).as_str(),
//             20.0,
//             20.0,
//             30.0,
//             WHITE,
//         );

//         next_frame().await;
//     }
// }
