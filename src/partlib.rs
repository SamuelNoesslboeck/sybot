use alloc::collections::BTreeMap;
use serde::Deserialize;

use stepper_lib::data::StepperConst;
use stepper_lib::data::servo::ServoConst;

pub type StepperPartLib = PartLib<StepperConst>;
pub type ServoPartLib = PartLib<ServoConst>;

pub type StdLibs = (StepperPartLib, ServoPartLib);


pub fn create_std_libs() -> StdLibs {
    let stepper_map = BTreeMap::from([
        ( "MOT_17HE15_1504S".to_string(), StepperConst::MOT_17HE15_1504S )
    ]);

    let servo_map = BTreeMap::from([
        ( "MG996R".to_string(), ServoConst::MG996R )
    ]);

    ( StepperPartLib::from(stepper_map), ServoPartLib::from(servo_map) ) 
}

pub fn scan_obj(libs : &StdLibs, obj : &mut serde_json::Value, repl : &String) {
    match obj.as_object_mut() { 
        // Check if object
        Some(obj) => {
            // Scan for fields
            for (k, v) in obj {
                if k == repl {
                    match v.as_str() {
                        Some(name) => { 
                            match libs.0.data_by_name(&String::from(name)) {
                                Some(data) => { 
                                    *v = serde_json::to_value(data).unwrap();
                                }
                                None => { 
                                    match libs.1.data_by_name(&String::from(name)) {
                                        Some(data) => { 
                                            *v = serde_json::to_value(data).unwrap();
                                        }
                                        None => { }
                                    };
                                }
                            };
                        },
                        None => { }
                    }
                } else {
                    scan_obj(libs, v, repl);
                }
            }
        }, 
        None => { }
    };

    match obj.as_array_mut() {
        Some(arr) => {
            for val in arr {
                scan_obj(libs, val, repl);
            }
        },
        None => { }
    }
}

pub struct PartLib<T> 
    where
        T: Clone, 
        T: PartialEq,
        T: for<'de> Deserialize<'de> {
    
    map : BTreeMap<String, T>
}

impl<T> PartLib<T>
    where
        T: Clone, 
        T: PartialEq,
        T: for<'de> Deserialize<'de> {
    
    pub fn new() -> Self {
        Self {
            map: BTreeMap::new()
        }
    }

    pub fn from(map : BTreeMap<String, T>) -> Self {
        Self { map }
    }

    pub fn add_map(&mut self, map : &mut BTreeMap<String, T>) {
        self.map.append(map)
    }

    pub fn add_json(&mut self, json : serde_json::Value) -> Result<(), crate::Error> {
        let mut map : BTreeMap<String, T> = match serde_json::from_value(json) {
            Ok(map) => map, 
            Err(err) => return Err(
                crate::Error::new(std::io::ErrorKind::InvalidData, format!("Parsing failed! {:?}", err)))
        };

        self.add_map(&mut map);
        Ok(())
    }

    // Load
        pub fn load_file(&mut self, file : &str) -> Result<(), crate::Error> {
            let json_str = std::fs::read_to_string(file)?;      // Returns std::io::Error as default Error
            let json : serde_json::Value = match serde_json::from_str(&json_str) {
                Ok(val) => val, 
                Err(err) => return Err(
                    crate::Error::new(std::io::ErrorKind::InvalidData, format!("Parsing failed! {:?}", err)))
            };

            self.add_json(json)?;
            Ok(())
        }
    //

    pub fn data_by_name(&self, name : &String) -> Option<T> {
        Some(self.map.get(name)?.clone())
    }

    pub fn name_by_data(&self, data : &T) -> Option<String> {
        for (k, v) in &self.map {
            if v == data {
                return Some(k.clone())
            }
        }

        None
    }
}