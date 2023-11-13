use core::fmt::Debug;

use glam::Vec3;
use syact::{SyncCompGroup, Tool, Setup, SimpleTool, SyncComp};
use syact::units::*;

// use crate::pkg::info::AngConf;
use crate::Descriptor;
use crate::conf::{Mode, AngConf};
use crate::rcs::Position;
use crate::remote::PushRemote;

// ####################
// #    SUBMODULES    #
// ####################
    mod stepper;
    pub use stepper::StepperRobot;
// 

// ##############
// #    VARS    #
// ##############
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
// 

// ###############
// #    ROBOT    #
// ###############
//
/// # `Robot` trait
/// 
/// The core trait that defines the properties of a robot. It is the first and core part of the R-D-S system, storing all the neccessary
/// data to drive multiple *synchronous* motors in the context of a robot.
/// 
/// A robot stores the follwing types of data
/// - `Vars` Robot variables (e.g. position)
/// - Current `Mode`, defines speed and safety
/// - `Tools` and tool currently selected
/// - Driving components (`SyncComp`) (e.g. motors, cylinders)
/// - `PushRemotes`
/// 
/// and provides the follwing functionalities
/// - `Gamma` / `Phi` distance conversion
pub trait Robot<G : SyncCompGroup<T, C>, T : SyncComp + ?Sized + 'static, const C : usize> : Setup {
    // Data
        /// Returns a reference to the robots variables
        fn vars(&self) -> &Vars<C>;

        /// Returns a mutable reference to the robots variables
        fn vars_mut(&mut self) -> &mut Vars<C>;

        fn ang_confs(&self) -> &[AngConf; C];

        /// Returns a reference to the component group of the robot
        fn comps(&self) -> &G;

        /// Returns a mutable reference to the component group of the robot 
        fn comps_mut(&mut self) -> &mut G;
    // 

    // Gamma & Phi - Distances
        /// All the angles used by the controls to represent the components extension/drive distance
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
        
        /// All the angles used by the controls to represent the mathematical angles
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

    // Synchronous movements
        fn move_j_sync(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<[Delta; C], crate::Error> {
            self.comps_mut().drive_rel(deltas, [speed_f; C])
        }

        fn move_abs_j_sync(&mut self, phis : [Phi; C], speed_f : f32) -> Result<[Delta; C], crate::Error> {
            let gammas = self.gammas_from_phis(phis);
            self.comps_mut().drive_abs(gammas, [speed_f; C])
        }

        fn move_p_sync(&mut self, desc : &mut dyn Descriptor<G, T, C>, p : Position, speed_f : f32) -> Result<[Delta; C], crate::Error>;
    // 
    
    // Asnychronous movement (complex movement)
        fn move_j(&mut self, deltas : [Delta; C], speed_f : f32) -> Result<(), crate::Error>;

        fn move_abs_j(&mut self, gammas : [Gamma; C], speed_f : f32) -> Result<(), crate::Error>;

        fn move_l(&mut self, desc : &mut dyn Descriptor<G, T, C>, distance : Vec3, accuracy : f32, speed : Omega) -> Result<(), crate::Error>;

        fn move_abs_l(&mut self, desc : &mut dyn Descriptor<G, T, C>, pos : Vec3, accuracy : f32, speed : Omega) -> Result<(), crate::Error>;

        fn move_p(&mut self, desc: &mut dyn Descriptor<G, T, C>, p : Position, speed_f : f32) -> Result<(), crate::Error>
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

    // Loads & Limits
        #[inline]
        fn apply_forces(&mut self, forces : &[Force; C]) -> Result<(), crate::Error> {
            self.comps_mut().apply_forces(forces)
        }

        #[inline]
        fn apply_inertias(&mut self, inertias : &[Inertia; C]) {
            self.comps_mut().apply_inertias(inertias)
        }

        fn set_limits(&mut self, min : &[Option<Gamma>; C], max : &[Option<Gamma>; C]) {
            self.comps_mut().set_limits(min, max)
        }

        fn set_omega_max(&mut self, omega_max : [Omega; C]) {
            self.comps_mut().set_omega_max(omega_max)
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
                let tool = self.get_tool_mut()
                    .ok_or("No tool has been equipped yet!")?;
                let simple_tool = tool.simple_tool_mut()
                    .ok_or("The tool equipped is no 'SimpleTool' (cannot be activated)")?;
                
                simple_tool.activate();

                Ok(simple_tool)
            } 

            fn deactivate_tool(&mut self) -> Result<&dyn SimpleTool, crate::Error> {
                let tool = self.get_tool_mut()
                    .ok_or("No tool has been equipped yet!")?;
                let simple_tool = tool.simple_tool_mut()
                    .ok_or("The tool equipped is no 'SimpleTool' (cannot be deactivated)")?;
                
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