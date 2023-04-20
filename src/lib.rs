#![doc = "
# sybot_lib

A simple library to control groups of components and robots.

Extension library for the [stepper_lib](https://crates.io/crates/stepper_lib).

# In action

The following example creates a new [SyArm robot](https://github.com/SamuelNoesslboeck/SyArm_Mk1), runs all setup functions and executes a GCode-script. 

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi
sybot_lib = { version = \"0.8.0\", features = [ \"rasp\" ] }

# ...
```

```rust
use sybot_lib::{Robot, ActRobot};
use sybot_lib::conf::JsonConfig;
use sybot_lib::intpr::Interpreter;
use sybot_lib::intpr::gcode::init_intpr;
use sybot_lib::robot::SyArm;

fn main() -> std::io::Result<()> {
    // Load the standard-partlibs in order to use motor names as data
    //
    // ```json
    // \"ctrl\": {
    //     \"consts\": \"MOT_17HE15_1504S\",    // Motor name, see
    // // <https://docs.rs/stepper_lib/0.11.1/stepper_lib/data/struct.StepperConst.html#associatedconstant.MOT_17HE15_1504S>
    //     \"pin_dir\": 17,
    //     \"pin_step\": 26
    // },
    // ```
    let libs = sybot_lib::partlib::create_std_libs();

    // Create the robot out of the [configuration file]
    // (https://github.com/SamuelNoesslboeck/sybot_lib/blob/master/res/SyArm_Mk1.conf.json)
    let mut syarm = SyArm::from_conf(
        JsonConfig::read_from_file(&libs, \"res/SyArm_Mk1.conf.json\")
    )?;

    // Run setup functions
    syarm.setup();
    // Enables async movements (multiple motors moving at once)
    syarm.setup_async();

    // DEBUG
        // Select \"NoTool\" at index 2
        syarm.set_tool_id(2);
    // 

    // Create a new GCode interpreter
    let intpr = init_intpr();

    // Run a GCode script
    dbg!(intpr.interpret_file(&mut syarm, \"res/gcode/basicYZpos.gcode\"));

    Ok(())
}
```
"]
#![crate_name = "sybot_lib"]
// #![deny(missing_docs)]

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

    pub mod partlib;

    /// Universal trait for input and output events happening in the robot. Used for 
    pub mod remote;

    pub mod robot;
    pub use robot::{ActRobot, BasicRobot, Robot};

    #[cfg(test)]
    mod tests;
//

// Types
/// Universal error type used in the crate
pub type Error = std::io::Error;
