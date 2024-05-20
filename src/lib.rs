#![doc = include_str!("../README.md")]
#![crate_name = "sybot"]
// #![deny(missing_docs)]

use syunit::*;

extern crate alloc;

// ####################
// #    SUBMODULES    #
// ####################
    /// Configurations for the robot in terms of position, speed and mode
    pub mod config;

    /// Quick and easy import of the library essentials
    pub mod prelude; 

    pub mod rcs;

    // ###################
    // #    SCRIPTING    #
    // ###################
    //
    /// Scripting of robots and more
    pub mod scr;
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

// Remotes
    pub enum PushMsg {
        Measurement,
        ToolChange
    }

    /// A `PushRemote` defines a remote connection that the robot can push values to
    pub trait PushRemote {
        /// Publish a set of phis to the remote connection
        fn push_phis(&mut self, phis : &[Phi]) -> Result<(), crate::Error>;

        fn push_other(&mut self, other : PushMsg) -> Result<(), crate::Error>;

        fn push_any(&mut self, msg_type : &str, msg : &[u8]) -> Result<(), crate::Error>;
    }
// 

// Interpreters

// 