#![crate_name = "sybot_lib"]
//! # SyBot Library
//! 
//! Control and calculation library various robots

// Module decleration
    mod arm;
    pub use arm::*;

    pub mod intpr;
    pub use intpr::init_intpr;

    mod robot;
    pub use robot::*;

    pub mod server;

    pub mod types;

    #[cfg(test)]
    mod tests;
//

// Public imports
pub use stepper_lib::{JsonConfig, MachineConfig};
pub use stepper_lib::gcode::Interpreter;