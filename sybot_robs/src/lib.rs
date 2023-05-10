extern crate alloc;

use glam::Vec3;
use stepper_lib::{SyncCompGroup, SyncComp, Tool};
use stepper_lib::units::*;

use sybot_pkg::RobotInfo;

pub type Error = Box<dyn std::error::Error>;

// TODO: REWORK VARS
#[derive(Clone, Debug)]
pub struct RobotVars {
    pub load : f32,
    
    pub f_speed : f32,

    pub decos : Vec<f32>,
    pub point : Vec3
}

impl Default for RobotVars {
    fn default() -> Self {
        Self {
            load: Default::default(),
            f_speed: 1.0,

            decos: Default::default(),
            point: Default::default()
        }
    }
}

/// A `PushRemote` defines a remote connection that the robot can push values to
pub trait PushRemote<const C : usize> {
    /// Publish a set of phis to the remote connection
    fn publ_phis(&mut self, phis : &[Phi; C]) -> Result<(), Error>;

    // fn pub_drive(&mut self);
}

pub trait BasicRobot {
    // Data
        fn info<'a>(&'a self) -> &'a RobotInfo;

        /// Returns a reference to the robots variables
        fn vars<'a>(&self) -> &'a RobotVars;

        /// Returns a mutable reference to the robots variables
        fn vars_mut<'a>(&mut self) -> &'a mut RobotVars;
    // 
}

pub trait Robot<const C : usize> : AsRef<dyn BasicRobot> + AsMut<dyn BasicRobot> {
    // Data
        /// Returns a reference to the component group of the robot
        fn comps(&self) -> &dyn SyncCompGroup<dyn SyncComp, C>;

        /// Returns a mutable reference to the component group of the robot 
        fn comps_mut(&mut self) -> &mut dyn SyncCompGroup<dyn SyncComp, C>;
    // 

    // Movements
        #[inline]
        fn move_j(&mut self, deltas : [Delta; C]) -> Result<[Delta; C], crate::Error> {
            let vels = self.max_vels();
            self.comps_mut().drive_rel(deltas, vels)
        }

        #[inline]
        fn move_j_abs(&mut self, gammas : [Gamma; C]) -> Result<[Delta; C], crate::Error> {
            let vels = self.max_vels();
            self.comps_mut().drive_abs(gammas, vels)
        }
    // 
}

pub trait CachedRobot<const C : usize> : AsRef<dyn Robot<C>> + AsMut<dyn Robot<C>> {

}

pub trait ToolRobot<const C : usize> : AsRef<dyn Robot<C>> + AsMut<dyn Robot<C>> {
    /// Returns a reference to the tool that is currently being used by the robot
    fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>>;

    /// Returns a mutable reference to the tool that is currently being used by the robot
    fn get_tool_mut(&mut self) -> Option<&mut Box<dyn Tool + std::marker::Send>>;

    /// Returns a reference to all the tools registered in the robot
    fn get_tools(&self) -> &Vec<Box<dyn Tool + std::marker::Send>>;

    /// Sets the id of the tool to be used and performs an automatic tool swap if necessary
    fn set_tool_id(&mut self, tool_id : usize) -> Option<&mut Box<dyn Tool + std::marker::Send>>;
}

pub trait RemoteRobot<const C : usize> : AsRef<dyn Robot<C>> + AsMut<dyn Robot<C>> {
    /// Adds a new remote to the robot
    fn add_remote(&mut self, remote : Box<dyn PushRemote<C> + 'static>);

    /// Returns a reference to all remotes of the robot
    fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote<C>>>;

    /// Returns a mutable reference to all remotes of the robot
    fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote<C>>>;
}