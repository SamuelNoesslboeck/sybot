use core::fmt::Debug;

use alloc::collections::BTreeMap;
use stepper_lib::{SyncComp, StepperCtrl, comp::{Cylinder, CylinderTriangle, GearJoint}};

use crate::CompInfo;

type JsonLib = BTreeMap<String, serde_json::Value>;
type CompLibFunc = fn(serde_json::Value) -> Result<Box<dyn SyncComp>, crate::Error>;
type CompLib = BTreeMap<String, CompLibFunc>;

pub struct PartLib {
    json_lib : JsonLib,
    comp_lib : CompLib
}

impl Debug for PartLib {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.json_lib.fmt(f)
    }
}

impl PartLib {
    pub fn new(json_lib : JsonLib, comp_lib : CompLib) -> Self {
        Self {
            json_lib,
            comp_lib
        }
    }

    pub fn std() -> Result<Self, crate::Error> {
        let stepper_ctrl_func : CompLibFunc = |val| { Ok(Box::new(serde_json::from_value::<StepperCtrl>(val)?)) };
        let cylinder_func : CompLibFunc = |val| { Ok(Box::new(serde_json::from_value::<Cylinder>(val)?)) };
        let cylinder_triangle_func : CompLibFunc = |val| { Ok(Box::new(serde_json::from_value::<CylinderTriangle>(val)?)) };
        let gearjoint_func : CompLibFunc = |val| { Ok(Box::new(serde_json::from_value::<GearJoint>(val)?)) };

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
        ])))
    }

    pub fn get(&self, res : &str) -> Option<&serde_json::Value> {
        self.json_lib.get(res)
    }

    pub fn load_comp(&self, info : &CompInfo) -> Result<Box<dyn SyncComp>, crate::Error> {
        if let Some(c_func) = self.comp_lib.get(&info.type_name) {
            Ok(c_func(info.obj.clone())?)
        } else {
            Err(format!("Component with typename '{}' not found!", info.type_name).into())
        }
    }
}