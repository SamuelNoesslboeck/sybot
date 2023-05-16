extern crate alloc;

use std::fs;
use std::path::Path;

use alloc::collections::BTreeMap;
use regex::Regex;
use serde::Deserialize;
use stepper_lib::data::LinkedData;

// Submodules
    mod info;
    pub use info::*;

    mod partlib;
    pub use partlib::*;

    mod pins;
    pub use pins::*;
    use sybot_rcs::WorldObj;
// 

pub type Error = Box<dyn std::error::Error>;

#[derive(Debug)]
pub struct Package {
    pub info : RobotInfo,
    
    pub lk : Option<LinkedData>,
    pub pins : Option<Pins>,
    pub comps : Option<BTreeMap<String, CompInfo>>,
    pub meas : Option<BTreeMap<String, MeasInfo>>, 
    
    pub tools : Option<Vec<ToolInfo>>,
    pub rcs_init : Option<WorldObj>,

    pub libs : PartLib
}

impl Package {
    pub fn new(info : RobotInfo) -> Result<Self, crate::Error> {
        Ok(Self {
            info,

            lk: None,
            pins: None,
            comps: None,
            meas: None,

            tools: None,
            rcs_init: None,

            libs: PartLib::std()?
        })
    }

    pub fn load<P : AsRef<Path>>(path : P) -> Result<Self, Error> {
        let comp_dir = path.as_ref().join("rob");
        let lib_dir = path.as_ref().join("lib");
        let rcs_dir = path.as_ref().join("rcs");

        let info = serde_json::from_str(&fs::read_to_string(path.as_ref().join("info.json"))?)?;

        let mut _self = Self::new(info)?;

        if comp_dir.exists() {
            _self.lk = _self.load_file(comp_dir.join("lk.json"))?;
            _self.pins = _self.load_file(comp_dir.join("pins.json"))?;
            _self.meas = _self.load_file(comp_dir.join("meas.json"))?;
            _self.comps = _self.load_file(comp_dir.join("comps.json"))?;
        }

        if lib_dir.exists() {
            _self.tools = _self.load_file(lib_dir.join("tools.json"))?;
        } 

        if rcs_dir.exists() {
            _self.rcs_init = _self.load_file(rcs_dir.join("init.json"))?;
        }

        Ok(_self)
    }

    pub fn load_file<T : for<'de> Deserialize<'de>, P : AsRef<Path>>(&self, path : P) -> Result<Option<T>, Error> {
        if let Ok(cont) = fs::read_to_string(path) {
            let mut j_cont : serde_json::Value = serde_json::from_str(&cont)?;

            self.replace_links(&mut j_cont)?;
            Ok(Some(serde_json::from_value::<T>(j_cont)?))
        } else {
            Ok(None)
        }
    }

    pub fn replace_links(&self, val : &mut serde_json::Value) -> Result<(), crate::Error> {
        match val {
            serde_json::Value::String(s) => {
                // Replace the link
                lazy_static::lazy_static! {
                    static ref LINK_REG : Regex = Regex::new(r"^<(.*?)://(.*?)>$").unwrap();
                }

                for cap in LINK_REG.captures_iter(s) {
                    let link = &cap[1];
                    let target = &cap[2];

                    *val = self.resolve_link(link, target)?;
                    break;      // Only one element if any
                }
            }, 
            serde_json::Value::Array(a) => {
                // Iterate through array
                for val in a {
                    self.replace_links(val)?;
                }
            },
            serde_json::Value::Object(o) => {
                // Iterate through object
                for (_, val) in o {
                    self.replace_links(val)?;
                }
            },
            _ => { }
        };
        Ok(())
    }

    pub fn resolve_link(&self, link : &str, target : &str) -> Result<serde_json::Value, crate::Error> {
        // println!("link: '{}'; target: {}", link, target);

        match link {
            "pin" => {
                if let Some(pins) = &self.pins {
                    if let Some(pin) = get_pin(pins, target.to_owned()) {
                        return Ok(serde_json::to_value(pin)?);
                    } 
                }
            }, 
            "meas" => {
                if let Some(meas_map) = &self.meas {
                    if let Some(meas) = meas_map.get(target) {
                        return Ok(serde_json::to_value(meas)?);
                    }
                }
            },
            "lib" => {
                if let Some(val) = self.libs.get(target) {
                    return Ok(val.clone());
                }
            }
            _ => { 
                return Err(format!("Link '<{}://{}>' could not be resolved ('{}' not found)", link, target, link).into());
            }
        }; 

        Err(format!("Link '<{}://{}>' could not be resolved ('{}' not found in '{}')", link, target, target, link).into())
    }

    pub fn parse_components(&self, comps )
}