use serde::{Serialize, Deserialize};
use syact::units::*;

use crate::pkg::{EmbeddedJsonInfo, GeneralInfo};

/// Extended json information for components
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompJsonInfo {
    info : GeneralInfo,
    #[serde(alias = "type")]
    type_name : Option<String>,
    obj : serde_json::Value,

    /// Information about angle conversions from phis to gammas
    #[serde(default)]
    pub ang : AngConf,
    /// Information about maximum speeds and distances
    #[serde(default)]
    pub limit : LimitInfo
}

impl EmbeddedJsonInfo for CompJsonInfo {
    fn info<'a>(&'a self) -> &'a GeneralInfo {
        &self.info
    }

    fn type_name<'a>(&'a self) -> Option<&'a String> {
        self.type_name.as_ref()
    }

    fn obj<'a>(&'a self) -> &'a serde_json::Value {
        &self.obj
    }

    fn split(self) -> (GeneralInfo, serde_json::Value) {
        ( self.info, self.obj )
    }
}

/// General information about the robot
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotInfo {
    /// Name of the robot
    pub name : String,
    /// Author of the robot (optional)
    pub author : Option<String>,
    /// Version of the robot configuration (optional)
    pub version : Option<String>,
    /// Additional properties
    pub props : serde_json::Value
}

// Sub-Structs
    /// Information about limits of a robot
    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct LimitInfo {
        /// Maximum angle of the component
        pub max : Option<Gamma>,
        /// Minium angle of the component
        pub min : Option<Gamma>,
        /// Maximum velocity of the component
        pub vel : Omega
    }
    
    /// Simulation information (e.g. mass, inertias, friction)
    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct SimInfo {
        /// Mass in kg 
        #[serde(default)]
        pub mass : Inertia,
        /// Friction in N / Nm
        #[serde(default)]
        pub fric : Force,
        /// Inertia in kg / kgm^2 
        #[serde(default)]
        pub inert : Inertia
    }
    
    /// Angle config (phi to gamma conversion)
    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct AngConf {
        /// Offset of the value
        #[serde(default)]
        pub offset : Delta,
        /// Wheiter or not the angle is a counterpart (negative addition)
        #[serde(default)]
        pub counter : bool
    }

    impl AngConf {
        /// Convert the given gamma angle to a phi angle
        pub fn phi_from_gamma(&self, gamma : Gamma) -> Phi {
            (if self.counter { 
                -gamma
            } else { 
                gamma
            } + self.offset).force_to_phi()
        }
        
        /// Convert the given phi angle to a gamma angle
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
    /// Parsing helper for segment movement
    #[derive(Debug, Clone, Copy, Serialize, Deserialize)]
    pub enum SegmentMovementInfo {
        /// Rotation around the Z axis
        Rotation,
        /// Linear movement in the direction of the vector
        Linear([f32; 3])
    }
// 

/// Information about a segment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SegmentInfo {
    /// General information about the segment
    pub info : GeneralInfo,
    /// Simulationinformation
    #[serde(default)]
    pub sim : SimInfo,
    /// Movement of the segment
    #[serde(alias = "move")]
    pub movement : SegmentMovementInfo
}    