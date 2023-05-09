use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize)]
pub struct RobotInfo {
    pub name : String,
    pub version : Option<String>,
    pub type_name : Option<String>,
    pub props : serde_json::Value
}