use glam::Vec3;
use stepper_lib::{SyncCompGroup, SyncComp, Tool};
use stepper_lib::units::*;

use crate::{Robot, BasicRobot};
use crate::conf::{JsonConfig, MachineConfig};
use crate::remote::PushRemote;
use crate::robot::{Points, Vectors, RobotVars};

pub trait ActRobot<const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> 
{
    // Types
        type Error : std::error::Error;
    // 

    // Inheritance
        fn brob(&self) -> &BasicRobot<COMP, DECO, DIM, ROT>;

        fn brob_mut(&mut self) -> &mut BasicRobot<COMP, DECO, DIM, ROT>;

        // Setup
            #[inline(always)]
            fn setup(&mut self) {
                self.brob_mut().setup()
            }

            #[inline(always)]
            fn setup_async(&mut self) {
                self.brob_mut().setup_async();
            }
        // 
    
        // Configuration
            /// Creates a new instance of the robot from a Json-Configuration file if it's format is appropriate
            fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error>
                where
                    Self: Sized;    
    
            /// Returns the 
            #[inline(always)]
            fn json_conf<'a>(&'a self) -> Option<&'a JsonConfig> {
                self.brob().json_conf()
            }
        // 
    
        // Stats and Data
            #[inline(always)]
            fn comps(&self) -> &dyn SyncCompGroup<dyn SyncComp, COMP> {
                self.brob().comps()
            }

            #[inline(always)]
            fn comps_mut(&mut self) -> &mut dyn SyncCompGroup<dyn SyncComp, COMP> {
                self.brob_mut().comps_mut()
            }
    
            #[inline(always)]
            fn vars(&self) -> &RobotVars<DECO> {
                self.brob().vars()
            }
            
            #[inline(always)]
            fn vars_mut(&mut self) -> &mut RobotVars<DECO> {
                self.brob_mut().vars_mut()
            }
            
            #[inline(always)]
            fn mach(&self) -> &MachineConfig<COMP, DIM, ROT> {
                self.brob().mach()
            }
            
            #[inline(always)]
            fn max_vels(&self) -> [Omega; COMP] {
                self.brob().max_vels()
            }   
            
            #[inline(always)]
            fn meas_deltas(&self) -> &[Delta; COMP] {
                self.brob().meas_deltas()
            }
        //
    
        // Positions
            #[inline(always)]
            fn home_pos(&self) -> &[Gamma; COMP] {
                self.brob().home_pos()
            }
            
            #[inline(always)]
            fn anchor(&self) -> &Vec3 {
                self.brob().anchor()
            }
        //
    
        // Tools
            /// Returns the current tool that is being used by the robot
            #[inline(always)]
            fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>> {
                self.brob().get_tool()
            }
            
            #[inline(always)]
            fn get_tool_mut(&mut self) -> Option<&mut Box<dyn Tool + std::marker::Send>> {
                self.brob_mut().get_tool_mut()
            }
            
            #[inline(always)]
            fn get_tools(&self) -> &Vec<Box<dyn Tool + std::marker::Send>> {
                self.brob().get_tools()
            }
            
            #[inline(always)]
            fn set_tool_id(&mut self, tool_id : usize) -> Option<&mut Box<dyn Tool + std::marker::Send>> {
                self.brob_mut().set_tool_id(tool_id)
            }
            
            #[inline(always)]
            fn gamma_tool(&self) -> Option<Gamma> {
                self.brob().gamma_tool()
            }
    
            // Actions 
            #[inline(always)]
            fn activate_tool(&mut self) -> Option<bool> {
                self.brob_mut().activate_tool()
            }
            
            #[inline(always)]
            fn activate_spindle(&mut self, cw : bool) -> Option<bool> {
                self.brob_mut().activate_spindle(cw)
            }
            
            #[inline(always)]
            fn deactivate_tool(&mut self) -> Option<bool> {
                self.brob_mut().deactivate_tool()
            }
            
            #[inline(always)]
            fn rotate_tool_abs(&mut self, gamma : Gamma) -> Option<Gamma> {
                self.brob_mut().rotate_tool_abs(gamma)
            }
        //
    
        // Remotes
            #[inline(always)]
            fn add_remote(&mut self, remote : Box<dyn PushRemote<COMP> + 'static>) {
                self.brob_mut().add_remote(remote) 
            }
            
            #[inline(always)]
            fn remotes<'a>(&'a self) -> &'a Vec<Box<dyn PushRemote<COMP>>> {
                self.brob().remotes()
            }
            
            #[inline(always)]
            fn remotes_mut<'a>(&'a mut self) -> &'a mut Vec<Box<dyn PushRemote<COMP>>> {
                self.brob_mut().remotes_mut()
            }
        // 
    //

    // Position
        /// Returns all the angles used by the controls to represent the components extension/drive distance
        #[inline(always)]
        fn gammas(&self) -> [Gamma; COMP] {
            self.comps().gammas()
        }

        /// Converts all angles (by subtracting an offset in most of the cases)
        fn gammas_from_phis(&self, phis : [Phi; COMP]) -> [Gamma; COMP];

        #[inline(always)]
        fn phis(&self) -> [Phi; COMP] {
            self.phis_from_gammas(self.gammas())
        }

        fn phis_from_gammas(&self, gammas : [Gamma; COMP]) -> [Phi; COMP];

        // Other
            fn deco_axis(&self) -> Vec3;
        //
    //

    // Calculation
        // Position
            #[inline]
            fn vecs_from_gammas(&self, gammas : &[Gamma; COMP]) -> Vectors<COMP> {
                self.vecs_from_phis(&self.phis_from_gammas(*gammas))
            }

            #[inline]
            fn points_from_gammas(&self, gammas : &[Gamma; COMP]) -> Points<COMP> {
                self.points_from_vecs(&self.vecs_from_gammas(gammas))
            }
            
            fn vecs_from_phis(&self, phis : &[Phi; COMP]) -> Vectors<COMP>;

            #[inline]
            fn points_from_phis(&self, phis : &[Phi; COMP]) -> Points<COMP> {
                self.points_from_vecs(&self.vecs_from_phis(phis))
            }

            #[inline]
            fn gammas_from_def_vec(&self, pos : Vec3) -> [Gamma; COMP] {
                self.gammas_from_phis(self.phis_from_def_vec(pos))
            }

            fn phis_from_def_vec(&self, pos : Vec3) -> [Phi; COMP];

            fn points_from_vecs(&self, vecs : &Vectors<COMP>) -> Points<COMP> {
                let mut points : Points<COMP> = [Vec3::ZERO; COMP];
                for i in 0 .. COMP {
                    points[i] += *self.anchor();
                    for n in 0 .. (i + 1) {
                        points[i] += vecs[n];
                    }
                }

                *points.last_mut().unwrap() += self.get_tool().unwrap().get_vec();
                points
            }

            fn vecs_from_points(&self, points : &Vectors<COMP>) -> Vectors<COMP> {
                let mut vecs : Points<COMP> = [Vec3::ZERO; COMP];
                vecs[0] = points[0] - *self.anchor();
                for i in 1 .. COMP {
                    vecs[i] = points[i] - points[i - 1];
                }
                vecs
            }

            fn reduce_to_def(&self, pos : Vec3, deco : [f32; DECO]) -> Vec3;

            fn phis_from_vec(&self, pos : Vec3, deco : [f32; DECO]) -> [Phi; COMP];

            // Current
            fn pos(&self) -> Vec3 {
                *self.points_from_phis(&self.phis()).last().unwrap()
            }
        //

        // Load
            #[inline]
            fn inertias_from_phis(&self, phis : &[Phi; COMP]) -> [Inertia; COMP] {
                self.inertias_from_vecs(&self.vecs_from_phis(phis))
            }

            #[inline]
            fn forces_from_phis(&self, phis : &[Phi; COMP]) -> [Force; COMP] {
                self.forces_from_vecs(&self.vecs_from_phis(phis))
            }

            fn inertias_from_vecs(&self, vecs : &Vectors<COMP>) -> [Inertia; COMP];

            fn forces_from_vecs(&self, vecs : &Vectors<COMP>) -> [Force; COMP];
        // 

        fn update(&mut self, phis : Option<&[Phi; COMP]>) -> Result<(), crate::Error>;
    //

    // Writing values
        #[inline]
        fn apply_inertias(&mut self, inertias : &[Inertia; COMP]) {
            self.comps_mut().apply_inertias(inertias);
        }

        #[inline]
        fn apply_forces(&mut self, forces : &[Force; COMP]) {
            self.comps_mut().apply_forces(forces);
        }

        #[inline]
        fn apply_bend_f(&mut self, f_bend : f32) {
            self.comps_mut().apply_bend_f(f_bend)
        }

        #[inline] 
        fn apply_speed_f(&mut self, f_speed : f32) {
            self.vars_mut().f_speed = f_speed;
        }

        // Position
            #[inline]
            fn write_gammas(&mut self, gammas : &[Gamma; COMP]) {
                self.comps_mut().write_gammas(gammas);
            }

            fn write_phis(&mut self, phis : &[Phi; COMP]) {
                let gammas = self.gammas_from_phis(*phis);
                self.comps_mut().write_gammas(&gammas)
            }
        // 
    // 

    // Movement
        #[inline]
        fn drive_rel(&mut self, deltas : [Delta; COMP]) -> Result<[Delta; COMP], stepper_lib::Error> {
            let vels = self.max_vels();
            self.comps_mut().drive_rel(deltas, vels)
        }

        #[inline]
        fn drive_abs(&mut self, gammas : [Gamma; COMP]) -> Result<[Delta; COMP], stepper_lib::Error> {
            let vels = self.max_vels();
            self.comps_mut().drive_abs(gammas, vels)
        }

        // Async 
        #[inline]
        fn drive_rel_async(&mut self, deltas : [Delta; COMP]) -> Result<(), stepper_lib::Error> {
            let vels = self.max_vels();
            self.comps_mut().drive_rel_async(deltas, vels)
        }
        
        #[inline]
        fn drive_abs_async(&mut self, gammas : [Gamma; COMP]) -> Result<(), stepper_lib::Error> {
            let vels = self.max_vels();
            self.comps_mut().drive_abs_async(gammas, vels)
        }

        // Single Component
            #[inline]
            fn drive_comp_rel(&mut self, index : usize, delta : Delta) -> Result<Delta, stepper_lib::Error> {
                let vels = self.max_vels();
                self.comps_mut()[index].drive_rel(delta, vels[index])
            }

            #[inline]
            fn drive_comp_abs(&mut self, index : usize, gamma : Gamma) -> Result<Delta, stepper_lib::Error> {
                let vels = self.max_vels();
                self.comps_mut()[index].drive_abs(gamma, vels[index])
            }

            #[inline]
            fn drive_comp_rel_async(&mut self, index : usize, delta : Delta) -> Result<(), stepper_lib::Error> {
                let vels = self.max_vels();
                self.comps_mut()[index].drive_rel_async(delta, vels[index])
            }

            #[inline]
            fn drive_comp_abs_async(&mut self, index : usize, gamma : Gamma) -> Result<(), stepper_lib::Error> {
                let vels = self.max_vels();
                self.comps_mut()[index].drive_abs_async(gamma, vels[index])
            }
        //

        // Measure
            fn measure(&mut self) -> Result<[Delta; COMP], stepper_lib::Error>;
        // 

        #[inline]
        fn await_inactive(&mut self) -> Result<[Delta; COMP], stepper_lib::Error> {
            self.comps_mut().await_inactive()
        }

        #[inline]
        fn set_end(&mut self, gammas : &[Gamma; COMP]) {
            self.comps_mut().set_ends(gammas)
        }

        fn set_limit(&mut self) {
            for i in 0 .. 4 {
                let min = self.mach().limit[i].min;
                let max = self.mach().limit[i].max;

                self.comps_mut()[i].set_limit(min, max);
            }
        }
    // 
}