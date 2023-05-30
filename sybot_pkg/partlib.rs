use core::fmt::Debug;

use alloc::collections::BTreeMap;
use stepper_lib::meas::EndSwitch;
use stepper_lib::prelude::SimpleMeas;
use stepper_lib::{SyncComp, StepperCtrl, Tool};
use stepper_lib::comp::{Cylinder, CylinderTriangle, GearJoint};
use stepper_lib::comp::tool::{AxialJoint, AxisTongs, PencilTool, Tongs};

use crate::{CompInfo, ToolInfo, MeasInfo};

type JsonLib = BTreeMap<String, serde_json::Value>;
type LibFunc<T> = fn(serde_json::Value) -> Result<T, crate::Error>;
type Lib<T> = BTreeMap<String, LibFunc<T>>;

macro_rules! dyn_lib {
    ( $ftype:ident, [ $( $name:ident ),* ] ) => {
        BTreeMap::from([
            $(( String::from(stringify!($name)), {
                    let func : LibFunc<Box<dyn $ftype>> = |val| { Ok(Box::new(serde_json::from_value::<$name>(val)?)) };
                    func
            }),)*
        ])
    };
}

pub struct PartLib {
    json_lib : JsonLib,
    comp_lib : Lib<Box<dyn SyncComp>>,
    tool_lib : Lib<Box<dyn Tool>>,
    meas_lib : Lib<Box<dyn SimpleMeas>>
}

impl Debug for PartLib {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.json_lib.fmt(f)
    }
}

impl PartLib {
    pub fn new(json_lib : JsonLib, comp_lib : Lib<Box<dyn SyncComp>>, 
        tool_lib : Lib<Box<dyn Tool>>, meas_lib : Lib<Box<dyn SimpleMeas>>) -> Self {
        Self {
            json_lib,
            comp_lib,
            tool_lib,
            meas_lib
        }
    }

    pub fn std() -> Result<Self, crate::Error> {
        Ok(Self::new(BTreeMap::from([
            // Steppers
            ( "stepper/MOT_17HE15_1504S".to_owned(), serde_json::to_value(stepper_lib::StepperConst::MOT_17HE15_1504S)? ),
            // Servo
            ( "servo/MG996R".to_owned(), serde_json::to_value(stepper_lib::data::servo::ServoConst::MG996R )? )
        ]), dyn_lib!(SyncComp, [ 
            StepperCtrl, Cylinder, CylinderTriangle, GearJoint
        ]), dyn_lib!(Tool, [
            AxialJoint, AxisTongs, PencilTool, Tongs
        ]), dyn_lib!(SimpleMeas, [
            EndSwitch
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

    pub fn parse_tool(&self, info : &ToolInfo) -> Result<Box<dyn Tool>, crate::Error> {
        if let Some(t_func) = self.tool_lib.get(&info.type_name) {
            Ok(t_func(info.obj.clone())?)
        } else {
            Err(format!("Tool with typename '{}' not found!", info.type_name).into())
        }
    }

    pub fn parse_meas(&self, info : &MeasInfo) -> Result<Box<dyn SimpleMeas>, crate::Error> {
        if let Some(t_func) = self.meas_lib.get(&info.sys) {
            Ok(t_func(info.obj.clone())?)
        } else {
            Err(format!("Measurement with typename '{}' not found!", info.sys).into())
        }
    }
}