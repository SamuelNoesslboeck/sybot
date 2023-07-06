#![doc = include_str!("../README.md")]
#![crate_name = "sybot"]

extern crate alloc;

pub use syact::{Setup, Tool, SyncComp};

// Module decleration
    /// Premade descriptors 
    pub mod desc;

    #[cfg(feature = "gcode")]
    pub use scr::gcode as gcode;
    
    /// Structures and methods for exposing the robot to the internet with a HTTP server 
    /// 
    /// # Features
    /// 
    /// Only available if the "http"-feature is available
    #[cfg(feature = "http")]
    pub use remote::http as http;

    /// Interpreters for sending text commands to control a [BasicRobot](crate::BasicRobot)
    // pub mod intpr;
    // pub use intpr::Interpreter;

    /// Structs and functions for calculating paths, loads and more
    pub use sybot_rcs::math as math;

    pub mod prelude; 

    /// Structures and methods for exposing the robot to the internet with a MQTT server
    /// 
    /// # Features 
    /// 
    /// Only available if the "mqtt"-feature is enabled
    #[cfg(feature = "mqtt")]
    pub use remote::mqtt as mqtt;

    mod remote;

    pub use sybot_pkg as pkg;
    pub use pkg::Package;

    pub use sybot_rcs as rcs;

    pub use sybot_robs as robs;
    pub use robs::*;

    pub use sybot_scr as scr;

    // #[cfg(test)]
    // mod tests;       TODO: Fix tests
//

// Types
/// Universal error type used in the crate
pub type Error = Box<dyn std::error::Error>;

// Lua
#[cfg(feature = "lua")]
use mlua::{Lua, Result, Table};

#[cfg(feature = "lua")]
#[mlua::lua_module]
fn sybot(lua : &Lua) -> Result<Table> {
    use core::cell::RefCell;

    use alloc::rc::Rc;
    use pkg::Package;

    let globals = lua.globals();

    std::env::set_var("RUST_BACKTRACE", "full");

    globals.set("load_rob", lua.create_function(|lua, path : String| {
        let globals = lua.globals();

        println!("Loading pkg ('{}'') ... ", path);

        let pkg = Package::load(path).unwrap();

        println!(" => Package loaded! "); 
        println!(" | - Info: {:?}", pkg.info);

        let mut syarm = SyArm::try_from(
            pkg
        ).unwrap();

        syarm.rob.setup().unwrap();
    
        globals.set("rob", scr::lua::RobStorage {
            rob: Rc::new(RefCell::new(syarm.rob)),
            desc: Rc::new(RefCell::new(syarm.desc))
        })?;     

        Ok(())
    })?)?;

    scr::lua::init_lib::<4>(lua)
}