use alloc::collections::BTreeMap;
use serde::{Serialize, Deserialize};
use serde::de::DeserializeOwned;

// Submodules
    pub mod info;

    mod pkg;
    pub use pkg::*;

    mod partlib;
    pub use partlib::*;
// 

/// Pins of the robot
pub type Pins = BTreeMap<String, BTreeMap<String, u8>>;

pub fn get_pin(pins : Option<&Pins>, path : &str) -> Option<u8> {
    let path_s : Vec<&str> = path.split('/').collect();

    if path_s.len() != 2 {
        panic!(" => Bad pin-path format! (<group>/<name>) [{:?}]", path_s);
    }
    
    if let Some(pins) = pins {
        if let Some(group) = pins.get(path_s[0]) {
            if let Some(pin) = group.get(path_s[1]) {
                return Some(*pin)
            } 
        }
    }

    None
}

/// General error type
pub type Error = Box<dyn std::error::Error>;

/// General information about a data structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneralInfo {
    /// Name of the data structure
    pub name: String,
    /// Description of the data structure
    pub desc: Option<String>
}

/// A data structure with general information and a json obj representating the actual structure
pub trait EmbeddedJsonInfo {
    /// Get general information about the data structure
    fn info(&self) -> &GeneralInfo;
    /// The typename of the data structure
    #[deprecated(since = "0.3.2", note = "Dynamic parsing will be removed in a future release.")]
    fn type_name(&self) -> Option<&String>;
    /// The json representation of the data structure
    fn obj(&self) -> &serde_json::Value;

    /// Parse the data structure from a json obj, leaving this json information untouched
    fn parse<T: for<'de> Deserialize<'de>>(&self) -> Result<T, crate::Error> {
        Ok(serde_json::from_value(self.obj().clone())?)
    }

    #[deprecated(since = "0.3.2", note = "Dynamic parsing will be removed in a future release.")]
    #[allow(deprecated)]
    /// Parse the data structure dynamically using the provided parsing table
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

    /// Split this structure into its components
    fn split(self) -> (GeneralInfo, serde_json::Value);

    /// Splits the data structure and parses the json structure
    fn split_and_parse<T : DeserializeOwned>(self) -> Result<(GeneralInfo, T), crate::Error> 
    where Self : Sized {
        let (info, obj) = self.split();
        Ok((info, serde_json::from_value(obj)?))
    }
}

/// A generic json data structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnyJsonInfo {
    info : GeneralInfo,
    #[serde(alias = "type")]
    type_name : Option<String>,
    obj : serde_json::Value
}

impl EmbeddedJsonInfo for AnyJsonInfo {
    fn info(&self) -> &GeneralInfo {
        &self.info
    }

    fn type_name(&self) -> Option<&String> {
        self.type_name.as_ref()
    }

    fn obj<'a>(&'a self) -> &'a serde_json::Value {
        &self.obj
    }

    fn split(self) -> (GeneralInfo, serde_json::Value) {
        ( self.info, self.obj )
    }
}

/// Parse an array of embedded json infos into an array of data structures
pub fn parse_array<I : EmbeddedJsonInfo, T: DeserializeOwned, const C : usize>(j_infos : Vec<I>) 
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

/// Parse an array embedded json infos into a structure
pub fn parse_struct<I : EmbeddedJsonInfo, T: DeserializeOwned>(j_infos : Vec<I>) 
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