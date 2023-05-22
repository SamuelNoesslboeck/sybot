use core::fmt::Debug;

use alloc::collections::BTreeMap;
use stepper_lib::{SyncComp, StepperCtrl, Tool};
use stepper_lib::comp::{Cylinder, CylinderTriangle, GearJoint};
use stepper_lib::comp::tool::{AxialJoint, AxisTongs, PencilTool, Tongs};

use crate::{CompInfo, ToolInfo};

type JsonLib = BTreeMap<String, serde_json::Value>;
type CompLibFunc = fn(serde_json::Value) -> Result<Box<dyn SyncComp>, crate::Error>;
type CompLib = BTreeMap<String, CompLibFunc>;
type ToolLibFunc = fn(serde_json::Value) -> Result<Box<dyn Tool + Send>, crate::Error>;
type ToolLib = BTreeMap<String, ToolLibFunc>;

pub struct PartLib {
    json_lib : JsonLib,
    comp_lib : CompLib,
    tool_lib : ToolLib
}

impl Debug for PartLib {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.json_lib.fmt(f)
    }
}

impl PartLib {
    pub fn new(json_lib : JsonLib, comp_lib : CompLib, tool_lib : ToolLib) -> Self {
        Self {
            json_lib,
            comp_lib,
            tool_lib
        }
    }

    pub fn std() -> Result<Self, crate::Error> {
        let stepper_ctrl_func : CompLibFunc = |val| { Ok(Box::new(serde_json::from_value::<StepperCtrl>(val)?)) };
        let cylinder_func : CompLibFunc = |val| { Ok(Box::new(serde_json::from_value::<Cylinder>(val)?)) };
        let cylinder_triangle_func : CompLibFunc = |val| { Ok(Box::new(serde_json::from_value::<CylinderTriangle>(val)?)) };
        let gearjoint_func : CompLibFunc = |val| { Ok(Box::new(serde_json::from_value::<GearJoint>(val)?)) };

        let axial_joint_func : ToolLibFunc = |val| { Ok(Box::new(serde_json::from_value::<AxialJoint>(val)?)) };
        let axis_tongs_func : ToolLibFunc = |val| { Ok(Box::new(serde_json::from_value::<AxisTongs>(val)?)) };
        let pencil_tool_func : ToolLibFunc = |val| { Ok(Box::new(serde_json::from_value::<PencilTool>(val)?)) };
        let tongs_tool_func : ToolLibFunc = |val| { Ok(Box::new(serde_json::from_value::<Tongs>(val)?)) };

        Ok(Self::new(BTreeMap::from([
            // Steppers
            ( "stepper/MOT_17HE15_1504S".to_owned(), serde_json::to_value(stepper_lib::StepperConst::MOT_17HE15_1504S)? ),
            // Servo
            ( "servo/MG996R".to_owned(), serde_json::to_value(stepper_lib::data::servo::ServoConst::MG996R )? )
        ]), BTreeMap::from([
            ( "StepperCtrl".to_owned(), stepper_ctrl_func ),
            ( "Cylinder".to_owned(), cylinder_func ),
            ( "CylinderTriangle".to_owned(), cylinder_triangle_func ),
            ( "GearJoint".to_owned(), gearjoint_func )
        ]), BTreeMap::from([
            ( "AxialJoint".to_owned(), axial_joint_func ),
            ( "AxisTongs".to_owned(), axis_tongs_func ),
            ( "PencilTool".to_owned(), pencil_tool_func ),
            ( "Tongs".to_owned(), tongs_tool_func )
        ])))
    }

    pub fn get(&self, res : &str) -> Option<&serde_json::Value> {
        self.json_lib.get(res)
    }

    pub fn parse_comp(&self, info : &CompInfo) -> Result<Box<dyn SyncComp>, crate::Error> {
        if let Some(c_func) = self.comp_lib.get(&info.type_name) {
            Ok(c_func(info.obj.clone())?)
        } else {
            Err(format!("Component with typename '{}' not found!", info.type_name).into())
        }
    }

    pub fn parse_tool(&self, info : &ToolInfo) -> Result<Box<dyn Tool + Send>, crate::Error> {
        if let Some(t_func) = self.tool_lib.get(&info.type_name) {
            Ok(t_func(info.obj.clone())?)
        } else {
            Err(format!("Tool with typename '{}' not found!", info.type_name).into())
        }
    }
}