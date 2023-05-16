use alloc::collections::BTreeMap;

#[derive(Debug)]
pub struct PartLib {
    lib : BTreeMap<String, serde_json::Value>
}

impl PartLib {
    pub fn new(lib : BTreeMap<String, serde_json::Value>) -> Self {
        Self {
            lib
        }
    }

    pub fn std() -> Result<Self, crate::Error> {
        Ok(Self::new(BTreeMap::from([
            // Steppers
            ( "stepper/MOT_17HE15_1504S".to_owned(), serde_json::to_value(stepper_lib::StepperConst::MOT_17HE15_1504S)? ),

            // Servo
            ( "servo/MG996R".to_owned(), serde_json::to_value(stepper_lib::data::servo::ServoConst::MG996R )? )
        ])))
    }

    pub fn get(&self, res : &str) -> Option<&serde_json::Value> {
        self.lib.get(res)
    }
}