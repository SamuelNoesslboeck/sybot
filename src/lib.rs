#![doc = include_str!("../README.md")]
#![crate_name = "sybot"]
// #![warn(missing_docs)]

extern crate alloc;

// ####################
// #    SUBMODULES    #
// ####################
    /// Configurations for the robot in terms of position, speed and mode
    pub mod config;

    // pub mod device;

    // #[cfg(feature = "gcode")]
    // pub use scr::gcode as gcode;
    
    // /// Structures and methods for exposing the robot to the internet with a HTTP server 
    // /// 
    // /// # Features
    // /// 
    // /// Only available if the "http"-feature is available
    // #[cfg(feature = "http")]
    // pub use remote::http as http;

    /// Interpreters for sending text commands to control a [BasicRobot](crate::BasicRobot)
    // pub mod intpr;
    // pub use intpr::Interpreter;

    /// Quick and easy import of the library essentials
    pub mod prelude; 

    // /// Structures and methods for exposing the robot to the internet with a MQTT server
    // /// 
    // /// # Features 
    // /// 
    // /// Only available if the "mqtt"-feature is enabled
    // #[cfg(feature = "mqtt")]
    // pub use remote::mqtt as mqtt;

    pub mod remote;
    pub use remote::PushRemote;

    // ##################
    // #    PACKAGES    #
    // ##################
    //
    // Packages are disabled currently, as a more static approach to the library is desired
    //
    // pub mod pkg;
    // pub use pkg::Package;

    pub mod rcs;

    // ###################
    // #    SCRIPTING    #
    // ###################
    //
    /// Scripting of robots and more
    pub mod scr;

    // #[cfg(test)]
    // mod tests;       TODO: Fix tests
//

// ########################
// #    R.D.S - SYSTEM    #
// ########################
    /// Everything related to the `Robot` trait
    pub mod robs;
    pub use robs::Robot;

    /// Everything related to the `Descriptor` trait
    pub mod desc;
    pub use desc::Descriptor;

    /// Everything related to the `Station` trait
    pub mod stat;
    pub use stat::Station;
// 

// ################
// #    ERRORS    #
// ################
    /// Universal error type used in the crate
    pub type Error = Box<dyn std::error::Error>;
//