use glam::Vec3; 

use stepper_lib::{Tool, JsonConfig, ComponentGroup, Omega, Gamma, Inertia, Force, Phi, Delta, MachineConfig};

// Submodules
mod safe;
pub use safe::*;
//

pub type Points<const COMP : usize> = [Vec3; COMP];
pub type Vectors<const COMP : usize> = [Vec3; COMP]; 


#[derive(Clone, Debug)]
pub struct RobotVars<const DECO : usize> {
    pub load : f32,

    pub decos : [f32; DECO],
    pub point : Vec3
}

impl<const DECO : usize> Default for RobotVars<DECO> {
    fn default() -> Self {
        Self {
            load: Default::default(),

            decos: [Default::default(); DECO],
            point: Default::default()
        }
    }
}

pub trait ConfRobot<const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> {
    // Configuration
        /// Creates a new instance of the robot from a Json-Configuration file if it's format is appropriate
        fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error>
            where
                Self: Sized;    

        /// Returns the 
        fn json_conf(&self) -> &Option<JsonConfig>;
    // 

    // Stats and Data
        fn comps(&self) -> &dyn ComponentGroup<COMP>;
        
        fn comps_mut(&mut self) -> &mut dyn ComponentGroup<COMP>;

        fn vars(&self) -> &RobotVars<DECO>;

        fn mach(&self) -> &MachineConfig<COMP, DIM, ROT>;

        fn max_vels(&self) -> &[Omega; COMP];
        
        fn meas_dists(&self) -> &[Delta; COMP];
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

        fn set_tool_id(&mut self, tool_id : usize);

        fn gamma_tool(&self) -> Option<Gamma>;

        // Actions 
        fn activate_tool(&mut self);

        fn activate_spindle(&mut self, cw : bool);

        fn deactivate_tool(&mut self);

        fn rotate_tool_abs(&mut self, gamma : Gamma);
    //
}

pub trait Robot<const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> : ConfRobot<COMP, DECO, DIM, ROT>
{
    // Types
        type Error : std::error::Error;
    // 

    // Position
        /// Returns all the angles used by the controls to represent the components extension/drive distance
        #[inline]
        fn all_gammas(&self) -> [Gamma; COMP] {
            self.comps().get_gammas()
        }

        /// Converts all angles (by subtracting an offset in most of the cases)
        fn gammas_from_phis(&self, phis : [Phi; COMP]) -> [Gamma; COMP];

        #[inline]
        fn all_phis(&self) -> [Phi; COMP] {
            self.phis_from_gammas(self.all_gammas())
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

                *points.last_mut().unwrap() += self.deco_axis();
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
                *self.points_from_phis(&self.all_phis()).last().unwrap()
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

        fn update(&mut self, phis : Option<&[Phi; COMP]>);
    //

    // Writing values
        #[inline]
        fn apply_load_inertias(&mut self, inertias : &[Inertia; COMP]) {
            self.comps_mut().apply_load_inertias(inertias);
        }

        #[inline]
        fn apply_load_forces(&mut self, forces : &[Force; COMP]) {
            self.comps_mut().apply_load_forces(forces);
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
        fn drive_rel(&mut self, deltas : [Delta; COMP]) -> [Delta; COMP] {
            let vels = *self.max_vels();
            self.comps_mut().drive_rel(deltas, vels)
        }

        #[inline]
        fn drive_abs(&mut self, gammas : [Gamma; COMP]) -> [Delta; COMP] {
            let vels = *self.max_vels();
            self.comps_mut().drive_abs(gammas, vels)
        }

        // Async 
        #[inline]
        fn drive_rel_async(&mut self, deltas : [Delta; COMP]) {
            let vels = *self.max_vels();
            self.comps_mut().drive_rel_async(deltas, vels);
        }
        
        #[inline]
        fn drive_abs_async(&mut self, gammas : [Gamma; COMP]) {
            let vels = *self.max_vels();
            self.comps_mut().drive_abs_async(gammas, vels);
        }

        // Single Component
            #[inline]
            fn drive_comp_rel(&mut self, index : usize, delta : Delta) -> Delta {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_rel(delta, vels[index])
            }

            #[inline]
            fn drive_comp_abs(&mut self, index : usize, gamma : Gamma) -> Delta {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_abs(gamma, vels[index])
            }

            #[inline]
            fn drive_comp_rel_async(&mut self, index : usize, delta : Delta) {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_rel_async(delta, vels[index])
            }

            #[inline]
            fn drive_comp_abs_async(&mut self, index : usize, gamma : Gamma) {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_abs_async(gamma, vels[index])
            }
        //

        // Measure
            fn measure(&mut self, acc : u64) -> Result<(), [bool; COMP]>;

            fn measure_async(&mut self, acc : u64);
        // 

        #[inline]
        fn await_inactive(&self) {
            self.comps().await_inactive()
        }

        #[inline]
        fn set_endpoint(&mut self, gammas : &[Gamma; COMP]) -> [bool; COMP] {
            self.comps_mut().set_endpoint(gammas)
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