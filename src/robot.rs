use glam::Vec3; 

use stepper_lib::{Tool, JsonConfig, ComponentGroup, Omega, Gamma, Inertia, Force, Phi, Delta};

// Submodules
mod safe;
pub use safe::*;
//

pub type Points<const N : usize> = [Vec3; N];
pub type Vectors<const N : usize> = [Vec3; N]; 


#[derive(Clone, Debug, Default)]
pub struct RobotVars
{
    pub load : f32,

    pub dec_angle : f32,
    pub point : Vec3
}

pub trait ConfRobot<const N : usize>
{
    // Configuration
        /// Creates a new instance of the robot from a Json-Configuration file if it's format is appropriate
        fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error>
            where
                Self: Sized;    

        /// Returns the 
        fn json_conf(&self) -> &Option<JsonConfig>;
    // 

    // Stats and Data
        fn comps(&self) -> &dyn ComponentGroup<N>;
        
        fn comps_mut(&mut self) -> &mut dyn ComponentGroup<N>;

        fn vars(&self) -> &RobotVars;

        fn max_vels(&self) -> &[Omega; N];
        
        fn meas_dists(&self) -> &[Delta; N];
    //

    // Positions
        fn home_pos(&self) -> &[Gamma; N];

        fn anchor(&self) -> &Vec3;
    //
}

pub trait Robot<const N : usize> : ConfRobot<N>
{
    // Types
        type Error : std::error::Error;
    // 

    // Position
        /// Returns all the angles used by the controls to represent the components extension/drive distance
        #[inline]
        fn all_gammas(&self) -> [Gamma; N] {
            self.comps().get_gammas()
        }

        /// Converts all angles (by subtracting an offset in most of the cases)
        fn gammas_from_phis(&self, phis : [Phi; N]) -> [Gamma; N];

        #[inline]
        fn all_phis(&self) -> [Phi; N] {
            self.phis_from_gammas(self.all_gammas())
        }

        fn phis_from_gammas(&self, gammas : [Gamma; N]) -> [Phi; N];

        // Other
            fn deco_axis(&self) -> Vec3;
        //
    //

    // Calculation
        // Position
            #[inline]
            fn vecs_from_gammas(&self, gammas : &[Gamma; N]) -> Vectors<N> {
                self.vecs_from_phis(&self.phis_from_gammas(*gammas))
            }

            #[inline]
            fn points_from_gammas(&self, gammas : &[Gamma; N]) -> Points<N> {
                self.points_from_vecs(&self.vecs_from_gammas(gammas))
            }
            
            fn vecs_from_phis(&self, phis : &[Phi; N]) -> Vectors<N>;

            #[inline]
            fn points_from_phis(&self, phis : &[Phi; N]) -> Points<N> {
                self.points_from_vecs(&self.vecs_from_phis(phis))
            }

            #[inline]
            fn gammas_from_def_vec(&self, pos : Vec3) -> [Gamma; N] {
                self.gammas_from_phis(self.phis_from_def_vec(pos))
            }

            fn phis_from_def_vec(&self, pos : Vec3) -> [Phi; N];

            fn points_from_vecs(&self, vecs : &Vectors<N>) -> Points<N> {
                let mut points : Points<N> = [Vec3::ZERO; N];
                for i in 0 .. N {
                    points[i] += *self.anchor();
                    for n in 0 .. (i + 1) {
                        points[i] += vecs[n];
                    }
                }
                points
            }

            fn vecs_from_points(&self, points : &Vectors<N>) -> Vectors<N> {
                let mut vecs : Points<N> = [Vec3::ZERO; N];
                vecs[0] = points[0] - *self.anchor();
                for i in 1 .. N {
                    vecs[i] = points[i] - points[i - 1];
                }
                vecs
            }

            fn reduce_to_def(&self, pos : Vec3, dec_ang : f32) -> Vec3;

            fn phis_from_vec(&self, pos : Vec3, dec_ang : f32) -> [Phi; N];
        //

        // Load
            #[inline]
            fn inertias_from_phis(&self, phis : &[Phi; N]) -> [Inertia; N] {
                self.inertias_from_vecs(&self.vecs_from_phis(phis))
            }

            #[inline]
            fn forces_from_phis(&self, phis : &[Phi; N]) -> [Force; N] {
                self.forces_from_vecs(&self.vecs_from_phis(phis))
            }

            fn inertias_from_vecs(&self, vecs : &Vectors<N>) -> [Inertia; N];

            fn forces_from_vecs(&self, vecs : &Vectors<N>) -> [Force; N];
        // 

        fn update(&mut self, phis : Option<&[Phi; N]>);
    //

    // Writing values
        #[inline]
        fn apply_load_inertias(&mut self, inertias : &[Inertia; N]) {
            self.comps_mut().apply_load_inertias(inertias);
        }

        #[inline]
        fn apply_load_forces(&mut self, forces : &[Force; N]) {
            self.comps_mut().apply_load_forces(forces);
        }

        #[inline]
        fn write_gammas(&mut self, gammas : &[Gamma; N]) {
            self.comps_mut().write_gammas(gammas);
        }
    // 

    // Movement
        #[inline]
        fn drive_rel(&mut self, deltas : [Delta; N]) -> [Gamma; N] {
            let vels = *self.max_vels();
            self.comps_mut().drive_rel(deltas, vels)
        }

        #[inline]
        fn drive_abs(&mut self, gammas : [Gamma; N]) -> [Gamma; N] {
            let vels = *self.max_vels();
            self.comps_mut().drive_abs(gammas, vels)
        }

        // Async 
        #[inline]
        fn drive_rel_async(&mut self, deltas : [Delta; N]) {
            let vels = *self.max_vels();
            self.comps_mut().drive_rel_async(deltas, vels);
        }
        
        #[inline]
        fn drive_abs_async(&mut self, gammas : [Gamma; N]) {
            let vels = *self.max_vels();
            self.comps_mut().drive_abs_async(gammas, vels);
        }

        // Single Component
            #[inline]
            fn drive_comp_rel(&mut self, index : usize, delta : Delta) -> Gamma {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_rel(delta, vels[index])
            }

            #[inline]
            fn drive_comp_abs(&mut self, index : usize, gamma : Gamma) -> Gamma {
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
            fn measure(&mut self, acc : u64) -> Result<(), [bool; N]>;

            fn measure_async(&mut self, acc : u64);
        // 

        #[inline]
        fn await_inactive(&self) {
            self.comps().await_inactive()
        }

        #[inline]
        fn set_endpoint(&mut self, gammas : &[Gamma; N]) -> [bool; N] {
            self.comps_mut().set_endpoint(gammas)
        }

        fn set_limit(&mut self);
    // 

    // Tools
        /// Returns the current tool that is being used by the robot
        fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>>;

        fn set_tool_id(&mut self, tool_id : usize);
    //
}