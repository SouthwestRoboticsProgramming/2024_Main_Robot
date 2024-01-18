use std::{error::Error, f64::consts::PI, time::Instant};

use geom::WindingDir;
use itertools::Itertools;
use lerp::Lerp;
use macroquad::prelude::*;
use math::Vec2f;
use obstacle::Obstacle;

mod config;
mod geom;
mod math;
mod obstacle;

fn draw_arc(center: Vec2f, radius: f64, min: f64, max: f64, thickness: f32, color: Color) {
    let mut prev_angle = max;
    for i in 0..=31 {
        let angle = min.lerp(max, (i as f64) / 31.0);

        let prev_pos = center + Vec2f::new_angle(radius, prev_angle);
        let pos = center + Vec2f::new_angle(radius, angle);

        draw_line(
            prev_pos.x as f32,
            prev_pos.y as f32,
            pos.x as f32,
            pos.y as f32,
            thickness,
            color,
        );

        prev_angle = angle;
    }
}

#[macroquad::main("Arc Pathfinding")]
async fn main() -> Result<(), Box<dyn Error>> {
    let conf: config::Config = config::load_or_save_default("config.json")?;
    let environment: config::EnvironmentConfig = config::load_or_save_default(&conf.env_file)?;

    let obstacles = environment
        .obstacles
        .values()
        .map(|(_, o)| o.clone().into_obstacle())
        .collect_vec();

    let field = geom::Field::generate(&obstacles, conf.robot_radius);

    let mut start = Vec2f::new(1.0, 1.0);
    let mut goal = Vec2f::new(1.2, 1.2);

    loop {
        clear_background(BLACK);

        let scale_x = (screen_width() - 50.0) / environment.width as f32;
        let scale_y = (screen_height() - 50.0) / environment.height as f32;

        let scale = scale_x.min(scale_y);
        let pixel = 1.0 / scale;
        set_camera(&Camera2D {
            zoom: Vec2 {
                x: 2.0 * scale / screen_width(),
                y: 2.0 * -scale / screen_height(),
            },
            target: Vec2 {
                x: environment.width as f32 / 2.0,
                y: environment.height as f32 / 2.0,
            },
            ..Default::default()
        });

        let (mouse_x, mouse_y) = mouse_position();
        let mouse_pos = Vec2f {
            x: ((mouse_x - screen_width() / 2.0) / scale) as f64 + environment.width / 2.0,
            y: ((screen_height() / 2.0 - mouse_y) / scale) as f64 + environment.height / 2.0,
        };
        draw_circle(mouse_pos.x as f32, mouse_pos.y as f32, 15.0 * pixel, BLUE);
        println!("Mouse pos: {:?}", mouse_pos);

        if is_mouse_button_down(MouseButton::Left) {
            start = mouse_pos;
        }
        if is_mouse_button_down(MouseButton::Right) {
            goal = mouse_pos;
        }

        for obstacle in &obstacles {
            match obstacle {
                Obstacle::Circle(circle) => {
                    draw_circle_lines(
                        circle.position.x as f32,
                        circle.position.y as f32,
                        circle.radius as f32,
                        2.0 * pixel,
                        ORANGE,
                    );
                }
                Obstacle::Polygon(polygon) => {
                    let mut prev = polygon.vertices[polygon.vertices.len() - 1];
                    for vert in &polygon.vertices {
                        draw_line(
                            prev.x as f32,
                            prev.y as f32,
                            vert.x as f32,
                            vert.y as f32,
                            2.0 * pixel,
                            ORANGE,
                        );
                        prev = *vert;
                    }
                }
            }
        }

        for arc in &field.arcs {
            let (min, max) = if arc.min_angle == arc.max_angle {
                (0.0, PI * 2.0)
            } else if arc.max_angle < arc.min_angle {
                (arc.min_angle, arc.max_angle + PI * 2.0)
            } else {
                (arc.min_angle, arc.max_angle)
            };

            draw_arc(arc.center, arc.radius, min, max, 2.0 * pixel, RED);
        }

        for segment in &field.segments {
            draw_line(
                segment.from.x as f32,
                segment.from.y as f32,
                segment.to.x as f32,
                segment.to.y as f32,
                2.0 * pixel,
                GRAY,
            );
        }

        for edge_set in &field.visibility {
            for edge in edge_set {
                let seg = &edge.segment;
                draw_line(
                    seg.from.x as f32,
                    seg.from.y as f32,
                    seg.to.x as f32,
                    seg.to.y as f32,
                    1.0 * pixel,
                    MAGENTA,
                );
            }
        }

        let calc_start = Instant::now();
        let path_opt = field.find_path(start, goal);
        // let path_opt: Option<Vec<geom::PathArc>> = None;
        let calc_end = Instant::now();

        if let Some(path) = path_opt {
            if !is_key_down(KeyCode::Space) {
                let mut p = start;
                for arc in path {
                    draw_line(
                        p.x as f32,
                        p.y as f32,
                        (arc.center.x + arc.radius * arc.incoming_angle.cos()) as f32,
                        (arc.center.y + arc.radius * arc.incoming_angle.sin()) as f32,
                        4.0 * pixel,
                        GREEN,
                    );

                    let mut angle1 = math::wrap_angle(arc.incoming_angle);
                    let mut angle2 = math::wrap_angle(arc.outgoing_angle);
                    if arc.direction == WindingDir::Clockwise {
                        std::mem::swap(&mut angle1, &mut angle2);
                    }

                    if angle2 < angle1 {
                        angle2 += PI * 2.0;
                    }

                    draw_arc(arc.center, arc.radius, angle1, angle2, 4.0 * pixel, GREEN);
                    p = arc.center + Vec2f::new_angle(arc.radius, arc.outgoing_angle);
                }
                draw_line(
                    p.x as f32,
                    p.y as f32,
                    goal.x as f32,
                    goal.y as f32,
                    4.0 * pixel,
                    GREEN,
                );
            }
        }

        let mut closest = None;
        for (id, arc) in field.arcs.iter().enumerate() {
            let dist = arc.center.distance_sq(mouse_pos).sqrt();
            if dist < arc.radius
                && match closest {
                    Some((_, distance)) => dist < distance,
                    None => true,
                }
            {
                closest = Some((id, dist));
            }
        }

        if let Some((id, _)) = closest {
            // Clockwise
            for edge in &field.visibility[id << 1] {
                draw_line(
                    edge.segment.from.x as f32,
                    edge.segment.from.y as f32,
                    edge.segment.to.x as f32,
                    edge.segment.to.y as f32,
                    8.0 * pixel,
                    BLUE,
                );
            }

            // Counterclockwise
            for edge in &field.visibility[(id << 1) + 1] {
                draw_line(
                    edge.segment.from.x as f32,
                    edge.segment.from.y as f32,
                    edge.segment.to.x as f32,
                    edge.segment.to.y as f32,
                    8.0 * pixel,
                    GOLD,
                );
            }
        }

        set_default_camera();
        let calc_time_ms = calc_end.duration_since(calc_start).as_secs_f64() * 1000.0;
        draw_text(
            format!("Calc time: {:.3} ms", calc_time_ms).as_str(),
            20.0,
            20.0,
            30.0,
            WHITE,
        );

        next_frame().await;
    }

    /*

    loop {
        let start, goal = read_request();

        let path_arcs = find_path(field, start, goal);
        let path_bezier = generate_bezier(path_arcs);

        send_result(path_bezier)
    }


     */
}
