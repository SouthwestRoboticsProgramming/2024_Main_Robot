// Positions of shapes are their centers

use serde::{Deserialize, Serialize};

use crate::vectors::Vec2f;

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Circle {
    pub position: Vec2f,
    pub radius: f64,
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Rectangle {
    pub position: Vec2f,
    pub size: Vec2f,
    pub rotation: f64,
    pub inverted: bool,
}

#[derive(Clone, Serialize, Deserialize, Debug)]
#[serde(tag = "type")]
pub enum CollisionShape {
    Circle(Circle),
    Rectangle(Rectangle),
}

pub struct AABB {
    pub min: Vec2f,
    pub max: Vec2f,
}

impl AABB {
    pub fn inflate(&self, radius: f64) -> AABB {
        AABB {
            min: Vec2f {
                x: self.min.x - radius,
                y: self.min.y - radius,
            },
            max: Vec2f {
                x: self.max.x + radius,
                y: self.max.y + radius,
            },
        }
    }
}

impl CollisionShape {
    pub fn collides_with_circle(&self, circle: &Circle) -> bool {
        match self {
            Self::Circle(c) => circle_circle_collides(circle, c),
            Self::Rectangle(r) => circle_rect_collides(circle, r),
        }
    }

    pub fn get_bounding_box(&self) -> AABB {
        match self {
            Self::Circle(c) => AABB {
                min: Vec2f {
                    x: c.position.x - c.radius,
                    y: c.position.y - c.radius,
                },
                max: Vec2f {
                    x: c.position.x + c.radius,
                    y: c.position.y + c.radius,
                },
            },
            Self::Rectangle(r) => {
                let sin = r.rotation.sin().abs();
                let cos = r.rotation.cos().abs();
                let half_sz = Vec2f {
                    x: (r.size.y * sin + r.size.x * cos) / 2.0,
                    y: (r.size.x * sin + r.size.y * cos) / 2.0,
                };

                AABB {
                    min: r.position - half_sz,
                    max: r.position + half_sz,
                }
            }
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
