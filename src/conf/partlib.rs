use std::collections::HashMap;
use serde::Deserialize;

pub struct PartLib<T> 
    where
        T: Clone, 
        T: for<'de> Deserialize<'de> {
    
    map : HashMap<String, T>
}

impl<T> PartLib<T>
    where
        T: Clone, 
        T: for<'de> Deserialize<'de> {
    
    pub fn new() -> Self {
        Self {
            map: HashMap::new()
        }
    }

    pub fn add_map(&mut self, map : &HashMap<String, T>) {
        for (k, v) in map {
            self.map.insert(k.clone(), v.clone());
        }
    }

    pub fn add_json(&mut self, json : serde_json::Value) {
        self.add_map(
            serde_json::from_value(json).unwrap()
        )
    }

    // Load
        pub fn load_file(&mut self, file : &str) {

        }
    // 
}