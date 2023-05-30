use serde::{Serialize, Deserialize};
use stepper_lib::units::*;

pub trait EmbeddedJsonInfo {
    fn obj<'a>(&'a self) -> &'a serde_json::Value;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotInfo {
    pub name : String,
    pub author : Option<String>,
    pub version : Option<String>,
    pub type_name : Option<String>,
    pub props : serde_json::Value
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct MeasInfo {
    pub name: String,
    pub sys : String, 
    pub obj : serde_json::Value
}

impl EmbeddedJsonInfo for MeasInfo {
    fn obj<'a>(&'a self) -> &'a serde_json::Value {
        &self.obj
    }
}

// Sub-Structs
    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct LimitInfo {
        pub max : Option<Gamma>,
        pub min : Option<Gamma>,
        pub vel : Omega
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct SimInfo {
        pub mass : Inertia,
        #[serde(default)]
        pub fric : Force,
        #[serde(default)]
        pub inert : Inertia
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct AngConf {
        #[serde(default)]
        pub offset : Delta,
        #[serde(default)]
        pub counter : bool
    }

    impl AngConf {
        pub fn phi_from_gamma(&self, gamma : Gamma) -> Phi {
            (if self.counter { 
                -gamma
            } else { 
                gamma
            } + self.offset).force_to_phi()
        }

        pub fn gamma_from_phi(&self, phi : Phi) -> Gamma {
            if self.counter { 
                -phi.force_to_gamma() + self.offset
            } else { 
                phi.force_to_gamma() - self.offset
            }
        }
    }
//

// Segments
    #[derive(Debug, Clone, Copy, Serialize, Deserialize)]
    pub enum SegmentMovementInfo {
        Rotation,
        Linear([f32; 3])
    }
// 

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompInfo {
    pub name : String, 
    pub type_name : String,
    pub obj : serde_json::Value,

    #[serde(default)]
    pub ang : AngConf,
    pub meas : Option<MeasInfo>,
    pub limit : LimitInfo
}

impl EmbeddedJsonInfo for CompInfo {
    fn obj<'a>(&'a self) -> &'a serde_json::Value {
        &self.obj
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolInfo {
    pub name : String,
    pub type_name : String,
    pub obj : serde_json::Value
}

impl EmbeddedJsonInfo for ToolInfo {
    fn obj<'a>(&'a self) -> &'a serde_json::Value {
        &self.obj
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SegmentInfo {
    pub name : String,
    pub sim : SimInfo,
    #[serde(alias = "move")]
    pub movement : SegmentMovementInfo
}