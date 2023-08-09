extern crate alloc;

use core::fmt::Debug;

use glam::Vec3;
use syact::{SyncCompGroup, Tool, Setup, SimpleTool, SyncComp};
use syact::units::*;

use crate::pkg::info::AngConf;
use crate::rcs::{WorldObj, Position, PointRef};
use crate::remote::PushRemote;

// Submodules
    mod device;
    pub use device::*;

    mod seg;
    pub use seg::*;

    mod stepper;
    pub use stepper::*;
// 

#[derive(Clone, Debug)]
pub struct Mode {
    pub name : String,
    pub desc : String,

    pub speed_f : f32
}

impl Mode {
    pub fn new<N : Into<String>, D : Into<String>>(name : N, desc : D, speed_f : f32) -> Self {
        Self { name : name.into(), desc: desc.into(), speed_f }
    }
}

pub fn default_modes() -> [Mode; 2] { 
    [
        Mode {
            name: String::from("Setup"), 
            desc: String::from("Mode with decreased speeds used for setting up"),
            speed_f : 0.5
        },
        Mode {
            name: String::from("Auto"), 
            desc: String::from("Mode used for running automated programms with full speed"),
            speed_f : 1.0
        }
    ]
}

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
        fn update<R : Robot<C, Comp = S, CompGroup = G>, S : SyncComp + ?Sized + 'static, G : SyncCompGroup<S, C>>(&mut self, rob : &mut R, phis : &[Phi; C]) -> Result<(), crate::Error>;
    // 

    // Calculation
        fn convert_pos<R : Robot<C, Comp = S, CompGroup = G>, S : SyncComp + ?Sized, G>(&mut self, rob : &mut R, pos : Position) -> Result<[Phi; C], crate::Error>;
    //

    // World object
        fn wobj<'a>(&'a self) -> &'a WorldObj;

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj;

        fn current_tcp(&self) -> &PointRef;

        /// Create a Vec3 from optional coordinates 
        fn cache_tcp(&self, x_opt : Option<f32>, y_opt : Option<f32>, z_opt : Option<f32>) -> Vec3;
    // 
}

pub trait Robot<const C : usize> : Setup {
    type Comp : SyncComp + ?Sized + 'static;
    type CompGroup : SyncCompGroup<Self::Comp, C>;

    // Data
        /// Returns a reference to the robots variables
        fn vars<'a>(&'a self) -> &'a Vars<C>;

        /// Returns a mutable reference to the robots variables
        fn vars_mut<'a>(&'a mut self) -> &'a mut Vars<C>;

        fn ang_confs<'a>(&'a self) -> &'a [AngConf];

        /// Returns a reference to the component group of the robot
        fn comps<'a>(&'a self) -> &'a Self::CompGroup;

        /// Returns a mutable reference to the component group of the robot 
        fn comps_mut<'a>(&'a mut self) -> &'a mut Self::CompGroup;
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
            self.comps_mut().drive_rel(deltas, [speed_f; C])
        }

        fn move_abs_j_sync(&mut self, phis : [Phi; C], speed_f : f32) -> Result<[Delta; C], crate::Error> {
            let gammas = self.gammas_from_phis(phis);
            self.comps_mut().drive_abs(gammas, [speed_f; C])
        }

        fn move_p_sync<D : Descriptor<C>>(&mut self, desc : &mut D, p : Position, speed_f : f32) -> Result<[Delta; C], crate::Error>;
    // 
    
    // Complex movement
        fn move_j(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<(), crate::Error>;

        fn move_abs_j(&mut self, gammas : [Gamma; C], speed_f : f32) -> Result<(), crate::Error>;

        fn move_l<D : Descriptor<C>>(&mut self, desc : &mut D, distance : Vec3, accuracy : f32, speed : Omega) -> Result<(), crate::Error>;

        fn move_abs_l<D : Descriptor<C>>(&mut self, desc : &mut D, pos : Vec3, accuracy : f32, speed : Omega) -> Result<(), crate::Error>;

        fn move_p<D : Descriptor<C>>(&mut self, desc : &mut D, p : Position, speed_f : f32) -> Result<(), crate::Error>
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
        fn get_tool(&self) -> Option<&dyn Tool>;

        /// Returns a mutable reference to the tool that is currently being used by the robot
        fn get_tool_mut(&mut self) -> Option<&mut dyn Tool>;

        /// Returns a reference to all the tools registered in the robot
        fn get_tools(&self) -> &Vec<Box<dyn Tool>>;

        /// Sets the id of the tool to be used and performs an automatic tool swap if necessary
        fn set_tool_id(&mut self, tool_id : Option<usize>) -> Option<&mut dyn Tool>;

        // Wrapper functions
            fn activate_tool(&mut self) -> Result<&dyn SimpleTool, crate::Error> {
                let tool = self.get_tool_mut().ok_or("No tool has been equipped yet!")?;
                let simple_tool = tool.simple_tool_mut().ok_or("The tool equipped is no 'SimpleTool' (cannot be activated)")?;
                
                simple_tool.activate();

                Ok(simple_tool)
            } 

            fn deactivate_tool(&mut self) -> Result<&dyn SimpleTool, crate::Error> {
                let tool = self.get_tool_mut().ok_or("No tool has been equipped yet!")?;
                let simple_tool = tool.simple_tool_mut().ok_or("The tool equipped is no 'SimpleTool' (cannot be deactivated)")?;
                
                simple_tool.deactivate();

                Ok(simple_tool)
            }
        // 
    // 

    // Modes
        fn mode(&self) -> &Mode;

        fn set_mode(&mut self, index : usize) -> Result<&Mode, crate::Error>;
    // 

    // Remote
        /// Adds a new remote to the robot
        fn add_remote(&mut self, remote : Box<dyn PushRemote>);

        /// Returns a reference to all remotes of the robot
        fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote>>;

        /// Returns a mutable reference to all remotes of the robot
        fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote>>;
    //

    // Events
        fn update(&mut self) -> Result<(), crate::Error>;
    // 
}

// For future releases 
// trait AsyncRobot {

// }