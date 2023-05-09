use std::fs;
use std::path::Path;

// Submodules
    mod elem;
    pub use elem::*;

    mod info;
    pub use info::*;

    mod pins;
    pub use pins::*;
// 

pub enum PinDecl {
    Num(u8),
    Path(String)
}

pub struct Package {
    pub info : RobotInfo,
    pub pins : Option<Pins>
}

impl Package {
    pub fn load<P : AsRef<Path>>(path : P) -> Result<Self, crate::Error> {
        let info = serde_json::from_str(&fs::read_to_string(path.as_ref().join("info.json"))?)?;
        let pins : Option<Pins> = if let Ok(pins_cont) = fs::read_to_string(path.as_ref().join("pins.json")) {
            Some(serde_json::from_str(&pins_cont)?)
        } else {
            None
        };

        Ok(Self {
            info,
            pins
        })
    }
}