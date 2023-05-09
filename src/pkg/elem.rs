use alloc::string::String;
use alloc::boxed::Box;

use serde::{Serialize, Deserialize};

use stepper_lib::{SyncComp, Tool};
use stepper_lib::units::*;

// Sub-Structs
    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct MeasInstance {
        pub pin : u8,
        pub set_val : Gamma,
        pub dist : Delta
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct LimitDecl {
        pub max : Option<Gamma>,
        pub min : Option<Gamma>,
        pub vel : Omega
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct SimData {
        pub mass : Inertia,
        #[serde(default)]
        pub fric : Force,
        #[serde(default)]
        pub inert : Inertia
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct AngleData {
        #[serde(default)]
        pub offset : Delta,
        #[serde(default)]
        pub counter : bool
    }
//

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigElement {
    #[serde(default)]
    pub name: String, 
    pub type_name : String,

    pub obj : serde_json::Value,

    #[serde(default)]
    pub ang : AngleData,
    #[serde(default)]
    pub sim : SimData,

    pub meas : Option<MeasInstance>,
    pub limit : Option<LimitDecl>
}

impl ConfigElement {
    pub fn get_comp(&self) -> Option<Box<dyn SyncComp + Send>> {
        match self.type_name.as_str() {
            "stepper_lib::ctrl::StepperCtrl" => Some(Box::new(
                    serde_json::from_value::<stepper_lib::StepperCtrl>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::cylinder::Cylinder" => Some(Box::new(
                serde_json::from_value::<stepper_lib::comp::Cylinder>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::cylinder_triangle::CylinderTriangle" => Some(Box::new(
                serde_json::from_value::<stepper_lib::comp::CylinderTriangle>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::gear_bearing::GearJoint" => Some(Box::new(
                serde_json::from_value::<stepper_lib::comp::GearJoint>(self.obj.clone()).unwrap()
            )), 
            _ => None
        }
    }

    pub fn get_tool(&self) -> Result<Box<dyn Tool + Send>, std::io::Error> {
        match self.type_name.as_str() {
            "stepper_lib::comp::tool::axial_joint::AxialJoint" => Ok(Box::new(
                serde_json::from_value::<stepper_lib::comp::tool::AxialJoint>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::tool::axis_tongs::AxisTongs" => Ok(Box::new(
                serde_json::from_value::<stepper_lib::comp::tool::AxisTongs>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::tool::no_tool::NoTool" => Ok(Box::new(
                stepper_lib::comp::tool::NoTool::new()
            )),
            "stepper_lib::comp::tool::pencil_tool::PencilTool" => Ok(Box::new(
                serde_json::from_value::<stepper_lib::comp::tool::PencilTool>(self.obj.clone()).unwrap()
            )),
            "stepper_lib::comp::tool::tongs::Tongs" => Ok(Box::new(
                serde_json::from_value::<stepper_lib::comp::tool::Tongs>(self.obj.clone()).unwrap()
            )),
            _ => Err(
                std::io::Error::new(std::io::ErrorKind::InvalidData, format!("The type name {:?} does not match any known type to the software", self.type_name))
            )
        }
    }
}

impl From<&Box<dyn SyncComp>> for ConfigElement {
    fn from(comp: &Box<dyn SyncComp>) -> Self {
        Self {
            name: String::new(),
            type_name: String::from(comp.get_type_name()),

            obj: comp.to_json().unwrap(),

            ang: Default::default(),
            sim: Default::default(),
            meas: None,
            limit: None
        }
    }
}

impl From<&Box<dyn Tool + Send>> for ConfigElement {
    fn from(tool: &Box<dyn Tool + Send>) -> Self {
        Self {
            name: String::new(),
            type_name: String::from(tool.get_type_name()),

            obj: tool.get_json(),

            ang: Default::default(),
            sim: Default::default(),
            meas: None,
            limit: None
        }
    }
}