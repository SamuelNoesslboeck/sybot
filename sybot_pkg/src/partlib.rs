use core::fmt::Debug;

use alloc::collections::BTreeMap;
use syact::{SyncComp, Stepper, Tool};
use syact::comp::{StepperCylinder, StepperCylTriangle, StepperGearJoint};
use syact::comp::tool::{AxialJoint, AxisTongs, PencilTool, Tongs};
use syact::meas::{EndSwitch, SimpleMeas, NoMeas};

use crate::EmbeddedJsonInfo;

pub type JsonLib = BTreeMap<String, serde_json::Value>;
pub type LibFunc<T> = fn(serde_json::Value) -> Result<T, crate::Error>;
pub type Lib<T> = BTreeMap<String, LibFunc<T>>;

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
            ( "stepper/MOT_17HE15_1504S".to_owned(), serde_json::to_value(syact::StepperConst::MOT_17HE15_1504S)? ),
            // Servo
            ( "servo/MG996R".to_owned(), serde_json::to_value(syact::data::servo::ServoConst::MG996R )? )
        ]), dyn_lib!(SyncComp, [ 
            Stepper, StepperCylinder, StepperCylTriangle, StepperGearJoint
        ]), dyn_lib!(Tool, [
            AxialJoint, AxisTongs, PencilTool, Tongs
        ]), dyn_lib!(SimpleMeas, [
            EndSwitch, NoMeas
        ])))
    }

    pub fn get(&self, res : &str) -> Option<&serde_json::Value> {
        self.json_lib.get(res)
    }

    #[allow(deprecated)]
    pub fn parse_comp_dyn<T : EmbeddedJsonInfo>(&self, info : &T) -> Result<Box<dyn SyncComp>, crate::Error> {
        info.parse_dyn(&self.comp_lib)
    }

    #[allow(deprecated)]
    pub fn parse_tool_dyn<T : EmbeddedJsonInfo>(&self, info : &T)-> Result<Box<dyn Tool>, crate::Error> {
        info.parse_dyn(&self.tool_lib)
    }
    
    #[allow(deprecated)]
    pub fn parse_meas_dyn<T : EmbeddedJsonInfo>(&self, info : &T) -> Result<Box<dyn SimpleMeas>, crate::Error> {
        info.parse_dyn(&self.meas_lib)
    }
}

impl Default for PartLib {
    fn default() -> Self {
        Self::std().unwrap()        // TODO: Rework
    }
}