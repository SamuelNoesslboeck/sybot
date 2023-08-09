use std::fs;
use std::path::Path;

use regex::Regex;
use serde::Deserialize;
use syact::prelude::SimpleMeas;
use syact::{CompData, Tool};

use crate::pkg::{AnyJsonInfo, Pins, PartLib, get_pin};
use crate::pkg::info::{RobotInfo, CompJsonInfo, SegmentInfo, AngConf};
use crate::rcs::WorldObj;

#[derive(Debug, Default)]
pub struct RobotPackage {
    pub data : Option<CompData>,
    pub comps : Option<Vec<CompJsonInfo>>,
    pub tools : Option<Vec<AnyJsonInfo>>,
    pub meas : Option<Vec<Vec<AnyJsonInfo>>>,

    pub libs : PartLib
}

impl RobotPackage {
    #[deprecated = "Dynamic parsing will be removed in future releases"]
    pub fn parse_tools_dyn(&self) -> Result<Vec<Box<dyn Tool>>, crate::Error> {
        let mut tools = vec![];

        if let Some(tinfos) = &self.tools {
            for info in tinfos {
                tools.push(self.libs.parse_tool_dyn(info)?);
            }
        }

        Ok(tools)
    }

    #[deprecated = "Dynamic parsing will be removed in future releases"]
    pub fn parse_meas_dyn(&self) -> Result<Vec<Vec<Box<dyn SimpleMeas>>>, crate::Error> {
        let mut meas = vec![];

        if let Some(minfos) = &self.meas {
            for info_vec in minfos {
                let mut meas_vec = vec![];

                for info in info_vec {
                    meas_vec.push(self.libs.parse_meas_dyn(info)?);
                }

                meas.push(meas_vec)
            }
        }

        Ok(meas)
    }

    #[deprecated = "Dynamic parsing will be removed in future releases"]
    pub fn parse_ang_confs(&self) -> Option<Vec<AngConf>> {
        let mut ang_confs = vec![];

        if let Some(cinfos) = &self.comps {
            for info in cinfos {
                ang_confs.push(info.ang);
            }

            Some(ang_confs)
        } else {
            None
        }
    }
}

#[derive(Debug, Default)]
pub struct DescPackage {
    pub rcs : Option<WorldObj>,
    pub segments : Option<Vec<SegmentInfo>>
}

#[derive(Debug, Default)]
pub struct StationPackage {
    // TODO: Add stations
}

#[derive(Debug)]
pub struct Package {
    pub info : RobotInfo, 
    pub pins : Option<Pins>,

    pub rob : RobotPackage,
    pub desc : DescPackage,
    pub stat : StationPackage,

    pub libs : PartLib
}

impl Package {
    // Construct / Destruct
        pub fn new(info : RobotInfo) -> Self {
            Self {
                info, 
                pins: None,

                rob: Default::default(),
                desc: Default::default(),
                stat: Default::default(),

                libs: PartLib::std().unwrap()
            }
        }

        pub fn unpack<R, D, S>(self) -> Result<(RobotInfo, R, D, S), crate::Error>
        where
            R : TryFrom<RobotPackage, Error = crate::Error>,
            D : TryFrom<DescPackage, Error = crate::Error>,
            S : TryFrom<StationPackage, Error = crate::Error>
        {
            Ok((
                self.info,
                R::try_from(self.rob)?,
                D::try_from(self.desc)?,
                S::try_from(self.stat)?
            ))
        }
    //

    // Loading
        pub fn load<P : AsRef<Path>>(path : P) -> Result<Self, crate::Error> {
            if !path.as_ref().exists() {
                return Err("Invalid path! No pkg found at the given path!".into()); 
            }

            let rob_dir = path.as_ref().join("rob");
            let desc_dir = path.as_ref().join("desc");

            let info_cont = match fs::read_to_string(path.as_ref().join("info.json")) {
                Ok(info) => info,
                Err(err) => return Err(format!("Error in reading info.json for pkg!\nError: {:?}", err).into())
            }; 
            let info = match serde_json::from_str(&info_cont) {
                Ok(info) => info,
                Err(err) => return Err(format!("Error in parsing info.json for pkg!\nError: {:?}", err).into())
            };

            let mut _self = Self::new(info);

            _self.pins = _self.load_file(&_self.libs, path.as_ref().join("pins.json"))?;

            if rob_dir.exists() {
                _self.rob.data = _self.load_file(&_self.libs, rob_dir.join("data.json"))?;
                _self.rob.meas = _self.load_file(&_self.libs, rob_dir.join("meas.json"))?;
                _self.rob.comps = _self.load_file(&_self.libs, rob_dir.join("comps.json"))?;
                _self.rob.tools = _self.load_file(&_self.libs, rob_dir.join("tools.json"))?;
            }

            // if lib_dir.exists() {
            //     _self.tools = _self.load_file(lib_dir.join("tools.json"))?;
            // } 

            if desc_dir.exists() {
                _self.desc.rcs = _self.load_file(&_self.libs, desc_dir.join("rcs.json"))?;
                _self.desc.segments = _self.load_file(&_self.libs, desc_dir.join("segments.json"))?;
            }

            // TODO: Station

            Ok(_self)
        }

        pub fn load_file<T : for<'de> Deserialize<'de>, P : AsRef<Path>>(&self, libs : &PartLib, path : P) -> Result<Option<T>, crate::Error> {
            if let Ok(cont) = fs::read_to_string(path.as_ref()) {
                let mut j_cont : serde_json::Value = serde_json::from_str(&cont).map_err(|err| -> crate::Error {
                    format!("Error parsing JSON for file '{:?}': {:?}", path.as_ref(), err).into()
                })?;
    
                self.replace_links(libs, &mut j_cont)?;
                Ok(Some(serde_json::from_value::<T>(j_cont).map_err(|err| -> crate::Error {
                    format!("Error converting JSON to value for file '{:?}': {:?}", path.as_ref(), err).into()
                })?))
            } else {
                Ok(None)
            }
        }

        fn replace_links(&self, libs : &PartLib, val : &mut serde_json::Value) -> Result<(), crate::Error> {
            match val {
                serde_json::Value::String(s) => {
                    // Replace the link
                    lazy_static::lazy_static! {
                        static ref LINK_REG : Regex = Regex::new(r"^<(.*?)://(.*?)>$").unwrap();
                    }
    
                    for cap in LINK_REG.captures_iter(s) {
                        let link = &cap[1];
                        let target = &cap[2];
    
                        *val = self.resolve_link(libs, link, target)?;
                        break;      // Only one element if any
                    }
                }, 
                serde_json::Value::Array(a) => {
                    // Iterate through array
                    for val in a {
                        self.replace_links(libs, val)?;
                    }
                },
                serde_json::Value::Object(o) => {
                    // Iterate through object
                    for (_, val) in o {
                        self.replace_links(libs, val)?;
                    }
                },
                _ => { }
            };
            Ok(())
        }

        pub fn resolve_link(&self, libs : &PartLib, link : &str, target : &str) -> Result<serde_json::Value, crate::Error> {
            match link {
                "pin" => {
                    if let Some(pin) = get_pin(self.pins.as_ref(), target) {
                        return Ok(serde_json::to_value(pin)?);
                    } 
                }, 
                "lib" => {
                    if let Some(val) = libs.get(target) {
                        return Ok(val.clone());
                    }
                }
                _ => { 
                    return Err(format!("Link '<{}://{}>' could not be resolved ('{}' not found)", link, target, link).into());
                }
            }; 
    
            Err(format!("Link '<{}://{}>' could not be resolved ('{}' not found in '{}')", link, target, target, link).into())
        }
    //  
}