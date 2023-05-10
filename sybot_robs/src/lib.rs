extern crate alloc;

use glam::Vec3;
use stepper_lib::{SyncCompGroup, SyncComp, Tool};
use stepper_lib::units::*;

use sybot_pkg::{RobotInfo, CompInfo};
use sybot_rcs::WorldObj;

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
        fn comp_infos<'a>(&'a self) -> &'a [CompInfo];

        /// Returns a reference to the component group of the robot
        fn comps<'a>(&'a self) -> &'a dyn SyncCompGroup<dyn SyncComp, C>;

        /// Returns a mutable reference to the component group of the robot 
        fn comps_mut<'a>(&'a mut self) -> &'a mut dyn SyncCompGroup<dyn SyncComp, C>;

        fn wobj<'a>(&'a self) -> &'a WorldObj;

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj;
    // 

    // Move data
        /// Returns all the angles used by the controls to represent the components extension/drive distance
        #[inline]
        fn gammas(&self) -> [Gamma; C] {
            self.comps().gammas()
        }

        /// Converts all angles (by adding offset and sometimes mirroring the value)
        fn gammas_from_phis(&self, phis : [Phi; C]) -> [Gamma; C] {
            let mut gammas = [Gamma::ZERO; C];
            let infos = self.comp_infos();

            for i in 0 .. C {
                gammas[i] = infos[i].ang.gamma_from_phi(phis[i]);
            }

            gammas
        }

        #[inline]
        fn phis(&self) -> [Phi; C] {
            self.phis_from_gammas(self.gammas())
        }

        /// Converts all teh angles (by adding offset and sometimes mirroring the value)
        fn phis_from_gammas(&self, gammas : [Gamma; C]) -> [Phi; C] {
            let mut phis = [Phi::ZERO; C];
            let infos = self.comp_infos();

            for i in 0 .. C {
                phis[i] = infos[i].ang.phi_from_gamma(gammas[i]);
            }

            phis
        }
    // 

    // Movements
        #[inline]
        fn move_j(&mut self, deltas : [Delta; C]) -> Result<[Delta; C], crate::Error> {
            self.comps_mut().drive_rel(deltas)
        }

        #[inline]
        fn move_j_abs(&mut self, gammas : [Gamma; C]) -> Result<[Delta; C], crate::Error> {
            self.comps_mut().drive_abs(gammas)
        }
    // 

    // Loads
        #[inline]
        fn apply_forces(&mut self, forces : &[Force; C]) {
            self.comps_mut().apply_forces(forces)
        }

        #[inline]
        fn apply_inertias(&mut self, inertias : &[Inertia; C]) {
            self.comps_mut().apply_inertias(inertias)
        }
    // 

    // Events
        fn pre_update(&mut self, phis : &[Phi; C]) -> Result<(), crate::Error> {
            self.update()
        }

        fn update(&mut self) -> Result<(), crate::Error>;
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