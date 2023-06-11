extern crate alloc;

use core::fmt::Debug;

use glam::Vec3;
use stepper_lib::{SyncCompGroup, Tool, Setup};
use stepper_lib::units::*;

use sybot_pkg::{RobotInfo, AngConf};
use sybot_rcs::{WorldObj, Position, PointRef};

pub type Error = Box<dyn std::error::Error>;

// Submodules
    mod device;
    pub use device::*;

    mod seg;
    pub use seg::*;

    mod structs;
    pub use structs::*;
// 

// TODO: REWORK VARS
#[derive(Clone, Debug)]
pub struct Vars<const C : usize> {
    pub phis : [Phi; C],
}

impl<const C : usize> Vars<C> {
    pub fn cache_phis(&self, phis_opt : [Option<Phi>; C]) -> [Phi; C] {
        let mut phis = self.phis;

        for i in 0 .. C {
            if let Some(phi) = phis_opt[i] {
                phis[i] = phi;
            }
        }

        phis
    }
}

impl<const C : usize> Default for Vars<C> {
    fn default() -> Self {
        Self {
            phis: [Phi::default(); C]
        }
    }
}

pub enum PushMsg {
    Measurement,
    ToolChange
}

/// A `PushRemote` defines a remote connection that the robot can push values to
pub trait PushRemote {
    /// Publish a set of phis to the remote connection
    fn push_phis(&mut self, phis : &[Phi]) -> Result<(), Error>;

    fn push_other(&mut self, other : PushMsg) -> Result<(), Error>;

    fn push_any(&mut self, msg_type : &str, msg : &[u8]) -> Result<(), Error>;
}

pub trait AxisConf {
    fn phis<'a>(&'a self) -> &'a [Phi];

    fn configure(&mut self, phis : Vec<Phi>) -> Result<(), crate::Error>; 
}

#[derive(Clone, Debug, Default)]
pub struct EmptyConf { }

impl AxisConf for EmptyConf {
    fn phis<'a>(&'a self) -> &'a [Phi] {
        &[]
    }

    fn configure(&mut self, _ : Vec<Phi>) -> Result<(), crate::Error> {
        Ok(())
    }
}

pub trait Descriptor<const C : usize> {
    // Axis conf
        fn aconf<'a>(&'a self) -> &'a dyn AxisConf;

        fn aconf_mut<'a>(&'a mut self) -> &'a mut dyn AxisConf;
    //

    // Events
        fn update(&mut self, rob : &mut dyn BasicRobot<C>, phis : &[Phi; C]) -> Result<(), crate::Error>;
    // 

    // Calculation
        fn convert_pos(&self, rob : &dyn BasicRobot<C>, pos : Position) -> Result<[Phi; C], crate::Error>;
    //

    // World object
        fn wobj<'a>(&'a self) -> &'a WorldObj;

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj;

        fn current_tcp(&self) -> &PointRef;

        /// Create a Vec3 from optional coordinates 
        fn cache_tcp(&self, x_opt : Option<f32>, y_opt : Option<f32>, z_opt : Option<f32>) -> Vec3;
    // 
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
        fn ang_confs<'a>(&'a self) -> &'a [AngConf];

        /// Returns a reference to the component group of the robot
        fn comps<'a>(&'a self) -> &'a dyn SyncCompGroup<C>;

        /// Returns a mutable reference to the component group of the robot 
        fn comps_mut<'a>(&'a mut self) -> &'a mut dyn SyncCompGroup<C>;
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
            let infos = self.ang_confs();

            for i in 0 .. C {
                gammas[i] = infos[i].gamma_from_phi(phis[i]);
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
            let infos = self.ang_confs();

            for i in 0 .. C {
                phis[i] = infos[i].phi_from_gamma(gammas[i]);
            }

            phis
        }

        fn valid_phis(&self, phis : &[Phi; C]) -> Result<(), crate::Error> {
            if self.comps().valid_gammas(
                &self.gammas_from_phis(*phis)
            ) {
                Ok(())
            } else {
                Err("The given phis are invalid!".into())
            }
        }
    // 

    // Movements
        fn set_limits(&mut self, min : &[Option<Gamma>; C], max : &[Option<Gamma>; C]) {
            self.comps_mut().set_limits(min, max)
        }

        fn set_omega_max(&mut self, omega_max : [Omega; C]) {
            self.comps_mut().set_omega_max(omega_max)
        }

        fn move_j_sync(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<[Delta; C], crate::Error> {
            self.comps_mut().drive_rel(deltas, speed_f)
        }

        fn move_abs_j_sync(&mut self, phis : [Phi; C], speed_f : f32) -> Result<[Delta; C], crate::Error> {
            let gammas = self.gammas_from_phis(phis);
            self.comps_mut().drive_abs(gammas, speed_f)
        }
 
        // fn move_p_sync(&mut self, desc : &mut dyn RobotDesc<C>, p : Vec3, speed_f : f32) -> Result<(), crate::Error> {
        //     self.move_j_abs_sync(

        //     )
        // }

        fn move_p_sync(&mut self, desc : &mut dyn Descriptor<C>, p : Position, speed_f : f32) -> Result<[Delta; C], crate::Error>;
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
        fn get_tool(&self) -> Option<&Box<dyn Tool>>;

        /// Returns a mutable reference to the tool that is currently being used by the robot
        fn get_tool_mut(&mut self) -> Option<&mut Box<dyn Tool>>;

        /// Returns a reference to all the tools registered in the robot
        fn get_tools(&self) -> &Vec<Box<dyn Tool>>;

        /// Sets the id of the tool to be used and performs an automatic tool swap if necessary
        fn set_tool_id(&mut self, tool_id : Option<usize>) -> Option<&mut Box<dyn Tool>>;
    // 

    // Remote
        /// Adds a new remote to the robot
        fn add_remote(&mut self, remote : Box<dyn PushRemote>);

        /// Returns a reference to all remotes of the robot
        fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote>>;

        /// Returns a mutable reference to all remotes of the robot
        fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote>>;
    //

    // Meas
        fn move_home(&mut self) -> Result<(), crate::Error>;
    //

    // Events
        fn update(&mut self) -> Result<(), crate::Error>;
    // 
}

pub trait ComplexRobot<const C : usize> : Setup + BasicRobot<C> + InfoRobot<C> {
    // Movement
        fn move_j(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<(), crate::Error> {
            self.comps_mut().drive_rel_async(deltas, speed_f)
        }

        fn move_abs_j(&mut self, gammas : [Gamma; C], speed_f : f32) -> Result<(), crate::Error> {
            self.comps_mut().drive_abs_async(gammas, speed_f)
        }

        fn move_l(&mut self, desc : &mut dyn Descriptor<C>, distance : Vec3, accuracy : f32, vel : Omega) -> Result<(), crate::Error>;

        fn move_abs_l(&mut self, desc : &mut dyn Descriptor<C>, pos : Vec3, accuracy : f32, vel : Omega) -> Result<(), crate::Error>;

        fn move_p(&mut self, desc: &mut dyn Descriptor<C>, p : Position, speed_f : f32) -> Result<(), crate::Error>
        where Self: Sized {
            let phis = desc.convert_pos(self, p)?;
            let gammas = self.gammas_from_phis(phis);
            self.move_abs_j(
                gammas,
                speed_f
            )
        }

        fn await_inactive(&mut self) -> Result<[Delta; C], crate::Error> {
            self.comps_mut().await_inactive()
        }
    // 
}

// For future releases 
// trait AsyncRobot {

// }