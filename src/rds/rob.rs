use glam::Vec3;
use syact::{SyncCompGroup, Tool, Setup, SimpleTool, SyncComp};
use syact::units::*;

use crate::{Descriptor, Mode};
use crate::pkg::info::AngConf;
use crate::rcs::Position;
use crate::remote::PushRemote;

use crate::rds::Vars;

// Robs
    mod stepper;
    pub use stepper::*;
// 

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
