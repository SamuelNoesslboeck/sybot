#![doc = include_str!("../README.md")]
#![crate_name = "sybot"]
// #![deny(missing_docs)]

use syact::math::movements::DefinedActuator;
use syact::{SyncActuator, SyncActuatorGroup};
use syunit::*;

extern crate alloc;

// ####################
// #    SUBMODULES    #
// ####################
    /// Configurations for the robot in terms of position, speed and mode
    pub mod config;

    /// Quick and easy import of the library essentials
    pub mod prelude; 

    /// RCS (Robot-Coordinate-System) module, manages the coordinate system and positions
    pub mod rcs;

    pub mod scr;

    #[cfg(test)]
    pub mod tests;
//

// ########################
// #    R.D.S - SYSTEM    #
// ########################
    /// Everything related to the `Robot` trait
    pub mod robs;
    pub use robs::{Robot, StepperRobot};

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
    /// Different types of events that can occur
    pub enum PushMsg {
        /// The robot has conducted a measurement
        Measurement,
        /// The robot has undergone a tool change
        ToolChange
    }

    /// A `PushRemote` defines a remote connection that the robot can push values to
    pub trait PushRemote {
        /// Publish a set of phis to the remote connection
        fn push_phis(&mut self, phis : &[Phi]) -> Result<(), crate::Error>;

        /// Publish a new `PushMsg`
        fn push_other(&mut self, other : PushMsg) -> Result<(), crate::Error>;

        /// Publish any type via bytes
        fn push_any(&mut self, msg_type : &str, msg : &[u8]) -> Result<(), crate::Error>;
    }
// 

// Interpreters
    /// Interpreters convert a string prompt into actions for the robot
    pub trait Interpreter<G, R, D, S, T, O, const C : usize> 
    where
        G : SyncActuatorGroup<T, C>,
        R : Robot<G, T, C>,
        D : Descriptor<C>,
        T : SyncActuator + DefinedActuator + ?Sized + 'static
    {
        /// Interpret a code string for a given robot
        fn interpret(&self, rob : &mut R, desc : &mut D, stat : &mut S, code : &str) -> O; 

        /// Interpret a file for a given robot
        fn interpret_file(&self, rob : &mut R, desc : &mut D, stat : &mut S, path : &str) -> O {
            self.interpret(rob, desc, stat, std::fs::read_to_string(path).unwrap().as_str())
        }
    }
// 