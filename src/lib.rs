#![doc = include_str!("../README.md")]
#![crate_name = "sybot_lib"]

extern crate alloc;

pub use stepper_lib::{Setup, Tool, SyncComp};

// Module decleration
    /// I/O for configuration files to parse whole robots out of JSON-text
    pub mod conf;
    pub use conf::{JsonConfig, MachineConfig};

    /// Structures and methods for exposing the robot to the internet with a HTTP server 
    /// 
    /// # Features
    /// 
    /// Only available if the "http"-feature is available
    #[cfg(feature = "http")]
    pub use remote::http;

    /// Interpreters for sending text commands to control a [BasicRobot](crate::BasicRobot)
    pub mod intpr;
    pub use intpr::Interpreter;

    /// Structs and functions for calculating paths, loads and more
    pub mod math;

    /// Structures and methods for exposing the robot to the internet with a MQTT server
    /// 
    /// # Features 
    /// 
    /// Only available if the "mqtt"-feature is enabled
    #[cfg(feature = "mqtt")]
    pub use remote::mqtt as mqtt;

    /// Partlibs help to simplify configuration files by storing standard-parts and motors
    pub mod partlib;

    pub use sybot_pkg as pkg;

    pub use sybot_rcs as rcs;

    /// Universal trait for input and output events happening in the robot. Used for 
    pub mod remote;

    /// Contains predefined robots and traits to describe them
    pub mod robot;
    pub use robot::{ActRobot, BasicRobot, Robot};

    pub use sybot_robs as robs;
    pub use robs::*;

    #[cfg(test)]
    mod tests;
//

// Types
/// Universal error type used in the crate
pub type Error = Box<dyn std::error::Error>;

// Lua lib
#[cfg(feature = "lua")]
use mlua::{Lua, Result, Table};

#[cfg(feature = "lua")]
#[mlua::lua_module]
fn sybot_lib(lua : &Lua) -> Result<Table> {
    intpr::lua::init_lib(lua)
}