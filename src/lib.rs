#![doc = include_str!("../README.md")]
#![crate_name = "sybot_lib"]

extern crate alloc;

// Module decleration
    /// I/O for configuration files to parse whole robots out of JSON-text
    pub mod conf;

    /// Structures and methods for exposing the robot to the internet with a HTTP server 
    /// 
    /// # Features
    /// 
    /// Only available if the "http"-feature is available
    #[cfg(feature = "http")]
    pub mod http;

    /// Interpreters for sending text commands to control a [BasicRobot](crate::BasicRobot)
    pub mod intpr;
    pub use intpr::Interpreter;

    /// Structures and methods for exposing the robot to the internet with a MQTT server
    /// 
    /// # Features 
    /// 
    /// Only available if the "mqtt"-feature is enabled
    #[cfg(feature = "mqtt")]
    pub mod mqtt;

    /// Partlibs help to simplify configuration files by storing standard-parts and motors
    pub mod partlib;

    /// Universal trait for input and output events happening in the robot. Used for 
    pub mod remote;

    /// Contains predefined robots and traits to describe them
    pub mod robot;
    pub use robot::{ActRobot, BasicRobot, Robot};

    #[cfg(test)]
    mod tests;
//

// Types
/// Universal error type used in the crate
pub type Error = std::io::Error;
