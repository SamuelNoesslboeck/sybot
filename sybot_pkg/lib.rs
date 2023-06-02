extern crate alloc;

use std::fs;
use std::path::Path;

use regex::Regex;
use serde::Deserialize;
use serde::de::DeserializeOwned;
use stepper_lib::prelude::SimpleMeas;
use stepper_lib::{SyncComp, Tool};
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
    pub cinfos : Option<Vec<CompJsonInfo>>,
    pub meas : Option<Vec<AnyJsonInfo>>, 
    
    pub tools : Option<Vec<AnyJsonInfo>>,
    pub devices : Option<Vec<AnyJsonInfo>>,
    pub wobj : Option<WorldObj>,
    pub segments : Option<Vec<SegmentInfo>>, 

    pub libs : PartLib
}

impl Package {
    pub fn new(info : RobotInfo) -> Result<Self, crate::Error> {
        Ok(Self {
            info,

            lk: None,
            pins: None,
            cinfos: None,
            meas: None,

            tools: None,
            devices: None,
            wobj: None,
            segments: None,

            libs: PartLib::std()?
        })
    }

    pub fn load<P : AsRef<Path>>(path : P) -> Result<Self, Error> {
        if !path.as_ref().exists() {
            return Err("Invalid path! No pkg found at the given path!".into()); 
        }

        let comp_dir = path.as_ref().join("rob");
        let lib_dir = path.as_ref().join("lib");
        let rcs_dir = path.as_ref().join("rcs");
        let stat_dir = path.as_ref().join("stat");

        let info_cont = match fs::read_to_string(path.as_ref().join("info.json")) {
            Ok(info) => info,
            Err(err) => return Err(format!("Error in reading info.json for pkg!\nError: {:?}", err).into())
        }; 
        let info = match serde_json::from_str(&info_cont) {
            Ok(info) => info,
            Err(err) => return Err(format!("Error in parsing info.json for pkg!\nError: {:?}", err).into())
        };

        let mut _self = Self::new(info)?;

        if stat_dir.exists() {
            _self.pins = _self.load_file(stat_dir.join("pins.json"))?;
            _self.devices = _self.load_file(stat_dir.join("devices.json"))?;
        }

        if comp_dir.exists() {
            _self.lk = _self.load_file(comp_dir.join("lk.json"))?;
            _self.meas = _self.load_file(comp_dir.join("meas.json"))?;
            _self.cinfos = _self.load_file(comp_dir.join("comps.json"))?;
            _self.segments = _self.load_file(comp_dir.join("segments.json"))?;
        }

        if lib_dir.exists() {
            _self.tools = _self.load_file(lib_dir.join("tools.json"))?;
        } 

        if rcs_dir.exists() {
            _self.wobj = _self.load_file(rcs_dir.join("init.json"))?;
        }

        Ok(_self)
    }

    fn load_file<T : for<'de> Deserialize<'de>, P : AsRef<Path>>(&self, path : P) -> Result<Option<T>, Error> {
        if let Ok(cont) = fs::read_to_string(path.as_ref()) {
            let mut j_cont : serde_json::Value = match serde_json::from_str(&cont) {
                Ok(value) => value,
                Err(err) => return Err(format!("Error parsing JSON for file '{:?}': {:?}", path.as_ref(), err).into())
            };

            self.replace_links(&mut j_cont)?;
            Ok(Some(match serde_json::from_value::<T>(j_cont) {
                Ok(value) => value,
                Err(err) => return Err(format!("Error converting JSON to value for file '{:?}': {:?}", path.as_ref(), err).into())
            }))
        } else {
            Ok(None)
        }
    }

    fn replace_links(&self, val : &mut serde_json::Value) -> Result<(), crate::Error> {
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

    pub fn parse<T : DeserializeOwned, I : EmbeddedJsonInfo>(&self, info : I) -> Result<T, crate::Error> {
        Ok(serde_json::from_value::<T>(info.obj().clone())?)
    } 

    pub fn parse_ang_confs(&self) -> Option<Vec<AngConf>> {
        let mut ang_confs = vec![];

        if let Some(cinfos) = &self.cinfos {
            for info in cinfos {
                ang_confs.push(info.ang);
            }

            Some(ang_confs)
        } else {
            None
        }
    }

    pub fn parse_comps_array<T : for<'de> Deserialize<'de>, const C : usize>(&self) -> Result<([T; C], Vec<GeneralInfo>), crate::Error> {
        if let Some(cinfos) = &self.cinfos {
            Ok(parse_array(cinfos.clone())?)
        } else {
            Err("No components found".into())
        }
    }

    pub fn parse_comps_struct<T : for<'de> Deserialize<'de>>(&self) -> Result<(T, Vec<GeneralInfo>), crate::Error> {
        if let Some(cinfos) = &self.cinfos {
            Ok(parse_struct(cinfos.clone())?)
        } else {
            Err("No components found".into())
        }
    }

    pub fn parse_comps_dyn<const C : usize>(&self) -> Result<[Box<dyn SyncComp>; C], crate::Error> {
        let mut comps = vec![];

        if let Some(cinfos) = &self.cinfos {
            for info in cinfos {
                comps.push(
                    self.libs.parse_comp_dyn(info)?
                )
            }
        }

        match comps.try_into() {
            Ok(c) => Ok(c),
            Err(v) => Err(
                format!("Not enough components defined for robot (Given: {}, Required: {})", v.len(), C).into()
            )
        }
    }

    pub fn parse_tools_dyn(&self) -> Result<Vec<Box<dyn Tool>>, crate::Error> {
        let mut tools = vec![];

        if let Some(tinfos) = &self.tools {
            for info in tinfos {
                tools.push(self.libs.parse_tool_dyn(info)?);
            }
        }

        Ok(tools)
    }

    pub fn parse_meas_dyn(&self) -> Result<Vec<Box<dyn SimpleMeas>>, crate::Error> {
        let mut meas = vec![];

        if let Some(minfos) = &self.meas {
            for info in minfos {
                meas.push(self.libs.parse_meas_dyn(info)?);
            }
        }

        Ok(meas)
    }

    pub fn req_wobj(self) -> Result<WorldObj, crate::Error> { 
        if let Some(wobj) = self.wobj {
            Ok(wobj)
        } else {
            Err("A valid world object must be provided".into())
        }
    }

    pub fn req_segments(self) -> Result<Vec<SegmentInfo>, crate::Error> {
        if let Some(segments) = self.segments {
            Ok(segments)
        } else {
            Err("A valid array of segments must be provided".into())
        }
    }

    pub fn req_desc(self) -> Result<(WorldObj, Vec<SegmentInfo>), crate::Error> {
        if let Some(wobj) = self.wobj {
            if let Some(segments) = self.segments {
                Ok((wobj, segments))
            } else {
                Err("A valid segments object must be provided".into())
            }
        } else {
            Err("A valid world object must be provided".into())
        }
    }
}