extern crate alloc;

use stepper_lib::{SyncCompGroup, SyncComp, Tool, Setup};
use stepper_lib::units::*;

use sybot_pkg::{RobotInfo, CompInfo};
use sybot_rcs::WorldObj;

pub type Error = Box<dyn std::error::Error>;

// Submodules
    mod structs;
    pub use structs::*;
// 

// TODO: REWORK VARS
#[derive(Clone, Debug)]
pub struct Vars<const C : usize> {
    pub phis : [Phi; C]
}

impl<const C : usize> Default for Vars<C> {
    fn default() -> Self {
        Self {
            phis: [Phi::default(); C]
        }
    }
}

/// A `PushRemote` defines a remote connection that the robot can push values to
pub trait PushRemote {
    /// Publish a set of phis to the remote connection
    fn publ_phis(&mut self, phis : &[Phi]) -> Result<(), Error>;

    // fn pub_drive(&mut self);
}

pub trait AxisConf {
    fn phis<'a>(&'a self) -> &'a [Phi];

    fn configure(&mut self, phis : Vec<Phi>); 
}

pub trait RobotDesc<const C : usize> : Setup {
    fn apply_aconf(&mut self, conf : Box<dyn AxisConf>); 
        
    fn aconf<'a>(&'a self) -> Option<&'a dyn AxisConf>;

    // Events
        fn update(&mut self, phis : &[Phi; C]) -> Result<(), crate::Error>;
    // 
        
    fn forces(&self, phis : &[Phi; C]) -> [Force; C];

    fn inertias(&self, phis : &[Phi; C]) -> [Inertia; C];
}

pub trait InfoRobot<const C : usize> {
    // Upcast
        fn basic_rob<'a>(&'a self) -> Option<&'a dyn BasicRobot<C>> { None }

        fn basic_rob_mut<'a>(&'a mut self) -> Option<&'a mut dyn BasicRobot<C>> { None }
    // 

    // Data
        fn info<'a>(&'a self) -> &'a RobotInfo;

        /// Returns a reference to the robots variables
        fn vars<'a>(&'a self) -> &'a Vars<C>;

        /// Returns a mutable reference to the robots variables
        fn vars_mut<'a>(&'a mut self) -> &'a mut Vars<C>;
    // 
}

pub trait BasicRobot<const C : usize> : Setup + InfoRobot<C> {
    // Upcast
        fn complex_rob<'a>(&'a self) -> Option<&'a dyn ComplexRobot<C>> { None }

        fn complex_rob_mut<'a>(&'a mut self) -> Option<&'a mut dyn ComplexRobot<C>> { None }
    // 

    // Data
        fn cinfos<'a>(&'a self) -> &'a [CompInfo];

        /// Returns a reference to the component group of the robot
        fn comps<'a>(&'a self) -> &'a dyn SyncCompGroup<dyn SyncComp, C>;

        /// Returns a mutable reference to the component group of the robot 
        fn comps_mut<'a>(&'a mut self) -> &'a mut dyn SyncCompGroup<dyn SyncComp, C>;
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
            let infos = self.cinfos();

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
            let infos = self.cinfos();

            for i in 0 .. C {
                phis[i] = infos[i].ang.phi_from_gamma(gammas[i]);
            }

            phis
        }

        fn valid_phis(&self, phis : [Phi; C]) -> Result<(), crate::Error>;
    // 

    // Movements
        #[inline]
        fn move_j_sync(&mut self, deltas : [Delta; C]) -> Result<[Delta; C], crate::Error> {
            self.comps_mut().drive_rel(deltas)
        }

        #[inline]
        fn move_abs_j_sync(&mut self, gammas : [Gamma; C]) -> Result<[Delta; C], crate::Error> {
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

    // Remote
        /// Adds a new remote to the robot
        fn add_remote(&mut self, remote : Box<dyn PushRemote + 'static>);

        /// Returns a reference to all remotes of the robot
        fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote>>;

        /// Returns a mutable reference to all remotes of the robot
        fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote>>;
    //

    // Events
        fn update(&mut self) -> Result<(), crate::Error>;
    // 
}

pub trait ComplexRobot<const C : usize> : Setup + BasicRobot<C> + InfoRobot<C> {
    // World object
        fn wobj<'a>(&'a self) -> &'a WorldObj;

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj;
    // 

    // Movement
        fn move_j(&mut self, deltas : [Delta; C]) -> Result<[Delta; C], crate::Error>;

        fn move_abs_j(&mut self, gammas : [Gamma; C]) -> Result<[Delta; C], crate::Error>;
    // 

    // Axis config
        #[inline]
        fn apply_aconf(&mut self, conf : Box<dyn AxisConf>) {
            if let Some(desc) = self.desc_mut() {
                desc.apply_aconf(conf)
            }
        }
        
        #[inline]
        fn aconf<'a>(&'a self) -> Option<&'a dyn AxisConf> {
            if let Some(desc) = self.desc() {
                desc.aconf()
            } else {
                None
            }
        }
    // 

    // Descriptor
        fn set_desc(&mut self, desc : dyn RobotDesc<C>);

        fn desc<'a>(&'a self) -> Option<&'a dyn RobotDesc<C>>;

        fn desc_mut<'a>(&'a self) -> Option<&'a mut dyn RobotDesc<C>>;
    // 
}