use glam::Vec3; 

use stepper_lib::SyncComp;
use stepper_lib::SyncCompGroup;
use stepper_lib::Tool;
use stepper_lib::units::*;

use crate::{JsonConfig, MachineConfig, PushRemote};

// Robots
mod arm;
pub use arm::SyArm;

mod omat;
pub use omat::Syomat;
//

// Submodules
mod act; 
pub use act::ActRobot;

mod safe;
pub use safe::SafeRobot;

mod vars;
pub use vars::RobotVars;
//

// Types
pub type Points<const COMP : usize> = [Vec3; COMP];
pub type Vectors<const COMP : usize> = [Vec3; COMP]; 


pub trait Robot<const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> {
    // Setup
        fn setup(&mut self); 

        fn setup_async(&mut self);
    // 

    // Configuration
        /// Creates a new instance of the robot from a Json-Configuration file if it's format is appropriate
        fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error>
            where
                Self: Sized;    

        /// Returns the 
        fn json_conf<'a>(&'a self) -> Option<&'a JsonConfig>;
    // 

    // Stats and Data
        fn comps(&self) -> &dyn SyncCompGroup<dyn SyncComp, COMP>;
        
        fn comps_mut(&mut self) -> &mut dyn SyncCompGroup<dyn SyncComp, COMP>;

        fn vars(&self) -> &RobotVars<DECO>;

        fn vars_mut(&mut self) -> &mut RobotVars<DECO>;

        fn mach(&self) -> &MachineConfig<COMP, DIM, ROT>;

        fn max_vels(&self) -> [Omega; COMP];
        
        fn meas_deltas(&self) -> &[Delta; COMP];
    //

    // Positions
        fn home_pos(&self) -> &[Gamma; COMP];

        fn anchor(&self) -> &Vec3;
    //

    // Tools
        /// Returns the current tool that is being used by the robot
        fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>>;

        fn get_tool_mut(&mut self) -> Option<&mut Box<dyn Tool + std::marker::Send>>;

        fn get_tools(&self) -> &Vec<Box<dyn Tool + std::marker::Send>>;

        fn set_tool_id(&mut self, tool_id : usize) -> Option<&mut Box<dyn Tool + std::marker::Send>>;

        fn gamma_tool(&self) -> Option<Gamma>;

        // Actions 
        fn activate_tool(&mut self) -> Option<bool>;

        fn activate_spindle(&mut self, cw : bool) -> Option<bool>;

        fn deactivate_tool(&mut self) -> Option<bool>;

        fn rotate_tool_abs(&mut self, gamma : Gamma) -> Option<Gamma>;
    //

    // Remotes
        fn add_remote(&mut self, remote : Box<dyn PushRemote<COMP> + 'static>);

        fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote<COMP>>>;

        fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote<COMP>>>;
    // 
}