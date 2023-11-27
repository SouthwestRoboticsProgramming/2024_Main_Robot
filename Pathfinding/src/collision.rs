// Positions of shapes are their centers

use serde::{Deserialize, Serialize};

use crate::vectors::Vec2f;

#[derive(Clone, Serialize, Deserialize)]
pub struct Circle {
    pub position: Vec2f,
    pub radius: f64,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Rectangle {
    pub position: Vec2f,
    pub size: Vec2f,
    pub rotation: f64,
    pub inverted: bool,
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CollisionShape {
    Circle(Circle),
    Rectangle(Rectangle),
}

impl CollisionShape {
    pub fn collides_with_circle(&self, circle: &Circle) -> bool {
        match self {
            Self::Circle(c) => circle_circle_collides(circle, c),
            Self::Rectangle(r) => circle_rect_collides(circle, r),
        }
    }
}

fn circle_circle_collides(a: &Circle, b: &Circle) -> bool {
    let collide_rad = a.radius + b.radius;
    (a.position - b.position).mag_sq() < collide_rad * collide_rad
}

fn circle_rect_collides(circle: &Circle, rect: &Rectangle) -> bool {
    let global_rel = circle.position - rect.position;
    let rel = global_rel.rotated(-rect.rotation);
    let half_sz = rect.size / 2.0;

    if rect.inverted {
        if rel.x.abs() > half_sz.x || rel.y.abs() > half_sz.y {
            return true;
        }

        let n = half_sz + rel;
        let p = half_sz - rel;

        let min = n.x.min(p.x).min(n.y.min(p.y));
        return min <= circle.radius;
    }

    let closest = Vec2f {
        x: rel.x.clamp(-half_sz.x, half_sz.x),
        y: rel.y.clamp(-half_sz.y, half_sz.y),
    };

    let delta = rel - closest;

    delta.mag_sq() < circle.radius * circle.radius
}
