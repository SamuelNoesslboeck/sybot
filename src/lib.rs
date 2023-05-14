#![doc = include_str!("../README.md")]
#![crate_name = "sybot_lib"]

extern crate alloc;

pub use stepper_lib::{Setup, Tool, SyncComp};

// Module decleration
    pub mod desc;
    
    /// Structures and methods for exposing the robot to the internet with a HTTP server 
    /// 
    /// # Features
    /// 
    /// Only available if the "http"-feature is available
    #[cfg(feature = "http")]
    pub use remote::http;

    /// Interpreters for sending text commands to control a [BasicRobot](crate::BasicRobot)
    // pub mod intpr;
    // pub use intpr::Interpreter;

    /// Structs and functions for calculating paths, loads and more
    pub mod math;

    /// Structures and methods for exposing the robot to the internet with a MQTT server
    /// 
    /// # Features 
    /// 
    /// Only available if the "mqtt"-feature is enabled
    #[cfg(feature = "mqtt")]
    pub use remote::mqtt as mqtt;

    mod remote;

    pub use sybot_pkg as pkg;

    pub use sybot_rcs as rcs;

    pub use sybot_robs as robs;
    pub use robs::*;

    #[cfg(test)]
    mod tests;
//

// Types
/// Universal error type used in the crate
pub type Error = Box<dyn std::error::Error>;