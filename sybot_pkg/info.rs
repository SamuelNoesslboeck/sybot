use serde::{Serialize, Deserialize};
use stepper_lib::units::*;

use crate::Lib;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneralInfo {
    pub name: String,
    pub desc: Option<String>
}

pub trait EmbeddedJsonInfo {
    fn info<'a>(&'a self) -> &'a GeneralInfo;
    fn type_name<'a>(&'a self) -> Option<&'a String>;
    fn obj<'a>(&'a self) -> &'a serde_json::Value;

    fn parse<'a, T: for<'de> Deserialize<'de>>(&self) -> Result<T, crate::Error> {
        Ok(serde_json::from_value(self.obj().clone())?)
    }

    fn parse_dyn<'a, T>(&self, func_set : &Lib<T>) -> Result<T, crate::Error> {
        if let Some(type_name) = self.type_name() {
            if let Some(c_func) = func_set.get(type_name) {
                Ok(c_func(self.obj().clone())?)
            } else {
                Err(format!("Component with typename '{}' not found!", type_name).into())
            }
        } else {
            Err("No type name found!".into())
        }
    }

    fn split(self) -> (GeneralInfo, serde_json::Value);
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnyJsonInfo {
    pub info : GeneralInfo,
    #[serde(alias = "type")]
    pub type_name : Option<String>,
    pub obj : serde_json::Value
}

impl EmbeddedJsonInfo for AnyJsonInfo {
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompJsonInfo {
    pub info : GeneralInfo,
    #[serde(alias = "type")]
    pub type_name : Option<String>,
    pub obj : serde_json::Value,
    #[serde(default)]
    pub ang : AngConf,
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

pub fn parse_array<I : EmbeddedJsonInfo, T: for<'de> Deserialize<'de>, const C : usize>(j_infos : Vec<I>) 
-> Result<([T; C], Vec<GeneralInfo>), crate::Error> {
    let mut infos = Vec::with_capacity(j_infos.len());
    let mut objs = Vec::with_capacity(j_infos.len());

    for j_info in j_infos {
        let (info, obj) = j_info.split();
        infos.push(info);
        objs.push(obj);
    }

    let array : [T; C] = match serde_json::from_value::<Vec<T>>(serde_json::Value::Array(objs))?.try_into() {
        Ok(array) => array,
        Err(_) => return Err("Wrong number of array elements!".into())
    }; 
 
    Ok((array, infos))
}

pub fn parse_struct<I : EmbeddedJsonInfo, T: for<'de> Deserialize<'de>>(j_infos : Vec<I>) 
-> Result<(T, Vec<GeneralInfo>), crate::Error> {
    let mut infos = Vec::with_capacity(j_infos.len());
    let mut objs = serde_json::Map::with_capacity(j_infos.len());

    for j_info in j_infos {
        let (info, obj) = j_info.split();
        objs.insert(info.name.clone(), obj);
        infos.push(info);
    }
 
    Ok((serde_json::from_value::<T>(serde_json::Value::Object(objs))?, infos))
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotInfo {
    pub name : String,
    pub author : Option<String>,
    pub version : Option<String>,
    pub type_name : Option<String>,
    pub props : serde_json::Value
}

// Sub-Structs
    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct LimitInfo {
        pub max : Option<Gamma>,
        pub min : Option<Gamma>,
        pub vel : Omega
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct SimInfo {
        #[serde(default)]
        pub mass : Inertia,
        #[serde(default)]
        pub fric : Force,
        #[serde(default)]
        pub inert : Inertia
    }

    #[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
    pub struct AngConf {
        #[serde(default)]
        pub offset : Delta,
        #[serde(default)]
        pub counter : bool
    }

    impl AngConf {
        pub fn phi_from_gamma(&self, gamma : Gamma) -> Phi {
            (if self.counter { 
                -gamma
            } else { 
                gamma
            } + self.offset).force_to_phi()
        }

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
    #[derive(Debug, Clone, Copy, Serialize, Deserialize)]
    pub enum SegmentMovementInfo {
        Rotation,
        Linear([f32; 3])
    }
// 

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SegmentInfo {
    pub info : GeneralInfo,
    #[serde(default)]
    pub sim : SimInfo,
    #[serde(alias = "move")]
    pub movement : SegmentMovementInfo
}