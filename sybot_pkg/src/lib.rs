extern crate alloc;

use std::fs;
use std::path::Path;

use alloc::collections::BTreeMap;
use stepper_lib::data::LinkedData;

// Submodules
    mod info;
    pub use info::*;

    mod pins;
    pub use pins::*;
// 

pub type Error = Box<dyn std::error::Error>;

#[derive(Debug)]
pub struct Package {
    pub info : RobotInfo,
    
    pub lk : Option<LinkedData>,
    pub pins : Option<Pins>,
    pub comps : Option<BTreeMap<String, CompInfo>>,
    pub meas : Option<BTreeMap<String, MeasInfo>>, 
    
    pub tools : Option<Vec<ToolInfo>>
}

impl Package {
    pub fn load<P : AsRef<Path>>(path : P) -> Result<Self, Error> {
        let comp_dir = path.as_ref().join("rob");
        let lib_dir = path.as_ref().join("lib");

        let info = serde_json::from_str(&fs::read_to_string(path.as_ref().join("info.json"))?)?;
        
        let mut lk : Option<LinkedData> = None;
        let mut pins : Option<Pins> = None;
        let mut comps : Option<BTreeMap<String, CompInfo>> = None;
        let mut meas : Option<BTreeMap<String, MeasInfo>> = None;
        
        let mut tools : Option<Vec<ToolInfo>> = None;

        if comp_dir.exists() {
            if let Ok(lk_cont) = fs::read_to_string(comp_dir.join("lk.json")) {
                lk = Some(serde_json::from_str(&lk_cont)?)
            }
    
            if let Ok(pins_cont) = fs::read_to_string(comp_dir.join("pins.json")) {
                pins = Some(serde_json::from_str(&pins_cont)?)
            } 

            if let Ok(comp_cont) = fs::read_to_string(comp_dir.join("comps.json")) {
                comps = Some(serde_json::from_str(&comp_cont)?)
            } 

            if let Ok(meas_cont) = fs::read_to_string(comp_dir.join("meas.json")) {
                meas = Some(serde_json::from_str(&meas_cont)?)
            } 
        }

        if lib_dir.exists() {
            if let Ok(tools_cont) = fs::read_to_string(lib_dir.join("tools.json")) {
                tools = Some(serde_json::from_str(&tools_cont)?)
            }
        }

        Ok(Self {
            info,

            lk,
            pins,
            comps,
            meas,

            tools
        })
    }
}