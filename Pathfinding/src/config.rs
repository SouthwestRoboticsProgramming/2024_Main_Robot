use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use tokio::fs;
use uuid::Uuid;

use crate::{collision, vectors::Vec2f};

#[derive(Serialize, Deserialize)]
pub struct Config {
    pub messenger: MessengerConfig,
    pub robot_radius: f64,
    pub field: FieldConfig,
    pub env_file: String,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            messenger: Default::default(),
            robot_radius: 0.5,
            field: Default::default(),
            env_file: "environment.json".to_string(),
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct MessengerConfig {
    pub host: String,
    pub port: u16,
    pub name: String,
}

impl Default for MessengerConfig {
    fn default() -> Self {
        Self {
            host: "localhost".to_string(),
            port: 5805,
            name: "Pathfinding".to_string(),
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct FieldConfig {
    pub cell_size: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

impl Default for FieldConfig {
    fn default() -> Self {
        Self {
            cell_size: 0.1524,
            origin_x: 0.0,
            origin_y: 1.0,
        }
    }
}

// ---------------------------------------

#[derive(Serialize, Deserialize)]
pub struct Environment {
    pub width: f64,
    pub height: f64,
    pub shapes: BTreeMap<Uuid, (String, collision::CollisionShape)>,
}

impl Default for Environment {
    fn default() -> Self {
        // By default include border rectangle
        let mut shapes = BTreeMap::new();
        shapes.insert(
            Uuid::new_v4(),
            (
                "Field perimeter".to_string(),
                collision::CollisionShape::Rectangle(collision::Rectangle {
                    position: Vec2f {
                        x: 8.2296,
                        y: 4.1148,
                    },
                    size: Vec2f {
                        x: 16.4592,
                        y: 8.2296,
                    },
                    rotation: 0.0,
                    inverted: true,
                }),
            ),
        );

        Self {
            width: 16.4592,
            height: 8.2296,
            shapes,
        }
    }
}

pub async fn load_or_save_default<T>(file_name: &str) -> Result<T, Box<dyn std::error::Error>>
where
    for<'a> T: Default + Serialize + Deserialize<'a>,
{
    let file_exists = match fs::try_exists(file_name).await {
        Ok(exists) => exists,
        Err(_) => false,
    };

    if file_exists {
        println!("Reading {}", file_name);
        let string = fs::read_to_string(file_name).await?;
        Ok(serde_json::from_str::<T>(&string)?)
    } else {
        println!("File {} not found, saving defaults", file_name);
        let def: T = Default::default();
        let string = serde_json::to_string_pretty(&def).unwrap();
        fs::write(file_name, &string).await?;
        Ok(def)
    }
}
