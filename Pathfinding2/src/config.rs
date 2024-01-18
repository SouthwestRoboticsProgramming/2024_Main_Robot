use std::{collections::BTreeMap, fs};

use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::{
    math::Vec2f,
    obstacle::{self, Obstacle},
};

#[derive(Serialize, Deserialize)]
pub struct Config {
    pub robot_radius: f64,
    pub env_file: String,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            robot_radius: 0.5,
            env_file: "environment.json".to_string(),
        }
    }
}

// ---------------------------------

#[derive(Serialize, Deserialize)]
pub struct EnvironmentConfig {
    pub obstacles: BTreeMap<Uuid, (String, ConfigObstacle)>,
}

impl Default for EnvironmentConfig {
    fn default() -> Self {
        let mut obstacles = BTreeMap::new();

        obstacles.insert(
            Uuid::new_v4(),
            (
                "Test circle".to_string(),
                ConfigObstacle::Circle(obstacle::Circle {
                    position: Vec2f::new(300.0, 300.0),
                    radius: 75.0,
                }),
            ),
        );

        Self { obstacles }
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Rectangle {
    pub position: Vec2f,
    pub size: Vec2f,
    pub rotation: f64,
}

#[derive(Serialize, Deserialize, Clone)]
#[serde(tag = "type")]
pub enum ConfigObstacle {
    Circle(obstacle::Circle),
    Polygon(obstacle::Polygon),
    Rectangle(Rectangle),
}

impl ConfigObstacle {
    pub fn into_obstacle(self) -> Obstacle {
        match self {
            Self::Circle(circle) => Obstacle::Circle(circle),
            Self::Polygon(polygon) => Obstacle::Polygon(polygon),
            Self::Rectangle(rect) => Obstacle::Polygon(obstacle::Polygon::new_rectangle(
                rect.position,
                rect.size,
                rect.rotation,
            )),
        }
    }
}

pub fn load_or_save_default<T>(file_name: &str) -> Result<T, Box<dyn std::error::Error>>
where
    for<'a> T: Default + Serialize + Deserialize<'a>,
{
    let file_exists = std::path::Path::new(file_name).exists();

    if file_exists {
        println!("Reading {}", file_name);
        let string = fs::read_to_string(file_name)?;
        Ok(serde_json::from_str::<T>(&string)?)
    } else {
        println!("File {} not found, saving defaults", file_name);
        let def: T = Default::default();
        let string = serde_json::to_string_pretty(&def).unwrap();
        fs::write(file_name, &string)?;
        Ok(def)
    }
}
