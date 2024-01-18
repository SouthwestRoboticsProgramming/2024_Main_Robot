use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

use crate::{
    geom::{Arc, Segment},
    math::{self, Vec2f},
};

#[derive(Serialize, Deserialize, Clone)]
pub struct Circle {
    pub position: Vec2f,
    pub radius: f64,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Polygon {
    pub vertices: Vec<Vec2f>,
}

impl Polygon {
    pub fn new_rectangle(pos: Vec2f, size: Vec2f, rotation: f64) -> Self {
        let half_w = size.x / 2.0;
        let half_h = size.y / 2.0;

        let mut vertices = Vec::with_capacity(4);
        vertices.push(pos + Vec2f::new(-half_w, -half_h).rotate(rotation));
        vertices.push(pos + Vec2f::new(half_w, -half_h).rotate(rotation));
        vertices.push(pos + Vec2f::new(half_w, half_h).rotate(rotation));
        vertices.push(pos + Vec2f::new(-half_w, half_h).rotate(rotation));

        Self { vertices }
    }
}

pub enum Obstacle {
    Circle(Circle),
    Polygon(Polygon),
}

impl Obstacle {
    pub fn convert_into(&self, inflate: f64, arcs: &mut Vec<Arc>, segments: &mut Vec<Segment>) {
        match self {
            Self::Circle(c) => convert_circle(c, inflate, arcs),
            Self::Polygon(p) => convert_polygon(p, inflate, arcs, segments),
        };
    }
}

fn convert_circle(circle: &Circle, inflate: f64, arcs: &mut Vec<Arc>) {
    arcs.push(Arc {
        center: circle.position,
        radius: circle.radius + inflate,
        min_angle: 0.0,
        max_angle: 0.0,
    });
}

fn convert_polygon(
    polygon: &Polygon,
    inflate: f64,
    arcs: &mut Vec<Arc>,
    segments: &mut Vec<Segment>,
) {
    let size = polygon.vertices.len();

    let mut prev = polygon.vertices[size - 1];
    for i in 0..size {
        let vertex = polygon.vertices[i];
        let delta = vertex - prev;

        let scale = inflate / delta.length();
        let offset = Vec2f {
            x: delta.y * scale,
            y: -delta.x * scale,
        };

        segments.push(Segment {
            from: vertex + offset,
            to: prev + offset,
        });

        let next = polygon.vertices[(i + 1) % size];
        let edge_angle = delta.angle();
        let next_angle = (next - vertex).angle();

        // Only add arc if the vertex is convex
        if math::floor_mod(next_angle - edge_angle, PI * 2.0) < PI {
            arcs.push(Arc::new(
                vertex,
                inflate,
                edge_angle - PI / 2.0,
                next_angle - PI / 2.0,
            ));
        }

        prev = vertex;
    }
}
