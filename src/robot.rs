use glam::Vec3; 

use stepper_lib::Setup;
use stepper_lib::SyncComp;
use stepper_lib::SyncCompGroup;
use stepper_lib::Tool;
use stepper_lib::units::*;

use crate::conf::{JsonConfig, MachineConfig};
use crate::remote::PushRemote;

// Submodules
mod basic;
pub use basic::BasicRobot;

mod segments;

mod vars;
pub use vars::RobotVars;

mod types;
//

/// A `PushRemote` defines a remote connection that the robot can push values to
pub trait PushRemote<const C : usize> {
    /// Publish a set of phis to the remote connection
    fn publ_phis(&mut self, phis : &[Phi; C]) -> Result<(), crate::Error>;

    // fn pub_drive(&mut self);
}

// Types
/// Positions of the robot represented with endpoints
pub type Points<const COMP : usize> = [Vec3; COMP];
/// Positions of the robot represented in a vector chain
pub type Vectors<const COMP : usize> = [Vec3; COMP]; 

/// A trait to define the basic data I/O that can be done with a robot
pub trait Robot<const C : usize> : Setup {
    // Setup
        /// Runs all the required functions to setup the async processes of the robot
        fn setup_async(&mut self);
    // 

    // Configuration
        /// Creates a new instance of the robot from a Json-Configuration file if it's format is appropriate
        fn from_conf(conf : JsonConfig, dim : usize, rot : usize) -> Result<Self, crate::Error>
        where
            Self: Sized;    

        /// Returns the `JsonConfig` of the robot
        fn json_conf<'a>(&'a self) -> Option<&'a JsonConfig>;
    // 

    // Stats and Data
        /// Returns a reference to the component group of the robot
        fn comps(&self) -> &dyn SyncCompGroup<dyn SyncComp, C>;

        /// Returns a mutable reference to the component group of the robot 
        fn comps_mut(&mut self) -> &mut dyn SyncCompGroup<dyn SyncComp, C>;

        /// Returns a reference to the robots variables
        fn vars(&self) -> &RobotVars;

        /// Returns a mutable reference to the robots variables
        fn vars_mut(&mut self) -> &mut RobotVars;

        /// Returns a reference to the robots machine configuration
        fn mach(&self) -> &MachineConfig<C>;

        /// Returns the maximum velocites each component can reach
        fn max_vels(&self) -> [Omega; C];
        
        /// Returns a reference to the measurement deltas
        fn meas_deltas(&self) -> &[Delta; C];
    //

    // Positions
        /// Returns a reference to the home position
        fn home_pos(&self) -> &[Gamma; C];

        /// Returns a reference to the anchor point of the robot
        fn anchor(&self) -> &Vec3;
    //

    // Tools
        /// Returns a reference to the tool that is currently being used by the robot
        fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>>;

        /// Returns a mutable reference to the tool that is currently being used by the robot
        fn get_tool_mut(&mut self) -> Option<&mut Box<dyn Tool + std::marker::Send>>;

        /// Returns a reference to all the tools registered in the robot
        fn get_tools(&self) -> &Vec<Box<dyn Tool + std::marker::Send>>;

        /// Sets the id of the tool to be used and performs an automatic tool swap if necessary
        fn set_tool_id(&mut self, tool_id : usize) -> Option<&mut Box<dyn Tool + std::marker::Send>>;
    //

    // Remotes
        /// Adds a new remote to the robot
        fn add_remote(&mut self, remote : Box<dyn PushRemote<C> + 'static>);

        /// Returns a reference to all remotes of the robot
        fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote<C>>>;

        /// Returns a mutable reference to all remotes of the robot
        fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote<C>>>;
    // 

    // Debug functions
        /// Prints out an overview of this robots configuration
        #[cfg(feature = "dbg-funcs")]
        fn print_conf_header(&self);
    // 
}