use glam::Vec3; 

use stepper_lib::{Tool, JsonConfig, ComponentGroup};
use stepper_lib::comp::{Gammas, Omegas, Inertias, Forces};

// Submodules
mod safe;
pub use safe::*;
//

pub type Phis<const N : usize> = Gammas<N>;

pub type Vectors<const N : usize> = [Vec3; N];
pub type Points<const N : usize> = [Vec3; N];


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

        fn max_vels(&self) -> &Omegas<N>;
        
        fn meas_dists(&self) -> &Gammas<N>;
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
        fn all_gammas(&self) -> Gammas<N> {
            self.comps().get_dist()
        }

        /// Converts all angles (by subtracting an offset in most of the cases)
        fn gammas_from_phis(&self, phis : Phis<N>) -> Gammas<N>;

        #[inline]
        fn all_phis(&self) -> Phis<N> {
            self.phis_from_gammas(self.all_gammas())
        }

        fn phis_from_gammas(&self, gammas : Gammas<N>) -> Phis<N>;

        // Other
            fn deco_axis(&self) -> Vec3;
            
            fn anchor(&self) -> &Vec3;

            fn home_pos(&self) -> &Gammas<N>;
        //
    //

    // Calculation
        // Position
            #[inline]
            fn vecs_from_gammas(&self, gammas : &Gammas<N>) -> Vectors<N> {
                self.vecs_from_phis(&self.phis_from_gammas(*gammas))
            }

            #[inline]
            fn points_from_gammas(&self, gammas : &Gammas<N>) -> Points<N> {
                self.points_from_vecs(&self.vecs_from_gammas(gammas))
            }
            
            fn vecs_from_phis(&self, phis : &Phis<N>) -> Vectors<N>;

            #[inline]
            fn points_from_phis(&self, phis : &Phis<N>) -> Points<N> {
                self.points_from_vecs(&self.vecs_from_phis(phis))
            }

            #[inline]
            fn gammas_from_def_vec(&self, pos : Vec3) -> Gammas<N> {
                self.gammas_from_phis(self.phis_from_def_vec(pos))
            }

            fn phis_from_def_vec(&self, pos : Vec3) -> Phis<N>;

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

            fn phis_from_vec(&self, pos : Vec3, dec_ang : f32) -> Phis<N>;
        //

        // Load
            #[inline]
            fn inertias_from_phis(&self, phis : &Phis<N>) -> Inertias<N> {
                self.inertias_from_vecs(&self.vecs_from_phis(phis))
            }

            #[inline]
            fn forces_from_phis(&self, phis : &Phis<N>) -> Forces<N> {
                self.forces_from_vecs(&self.vecs_from_phis(phis))
            }

            fn inertias_from_vecs(&self, vecs : &Vectors<N>) -> Inertias<N>;

            fn forces_from_vecs(&self, vecs : &Vectors<N>) -> Forces<N>;
        // 

        fn update(&mut self, phis : Option<&Phis<N>>);
    //

    // Writing values
        #[inline]
        fn apply_load_inertias(&mut self, inertias : &Inertias<N>) {
            self.comps_mut().apply_load_inertias(inertias);
        }

        #[inline]
        fn apply_load_forces(&mut self, forces : &Forces<N>) {
            self.comps_mut().apply_load_forces(forces);
        }

        #[inline]
        fn write_gammas(&mut self, gammas : &Gammas<N>) {
            self.comps_mut().write_dist(gammas);
        }
    // 

    // Movement
        #[inline]
        fn drive_rel(&mut self, dist : [f32; N]) -> [f32; N] {
            let vels = *self.max_vels();
            self.comps_mut().drive_rel(dist, vels)
        }

        #[inline]
        fn drive_abs(&mut self, dist : Gammas<N>) -> [f32; N] {
            let vels = *self.max_vels();
            self.comps_mut().drive_abs(dist, vels)
        }

        // Async 
        #[inline]
        fn drive_rel_async(&mut self, dist : [f32; N]) {
            let vels = *self.max_vels();
            self.comps_mut().drive_rel_async(dist, vels);
        }
        
        #[inline]
        fn drive_abs_async(&mut self, dist : Gammas<N>) {
            let vels = *self.max_vels();
            self.comps_mut().drive_abs_async(dist, vels);
        }

        // Single Component
            #[inline]
            fn drive_comp_rel(&mut self, index : usize, dist : f32) -> f32 {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_rel(dist, vels[index])
            }

            #[inline]
            fn drive_comp_abs(&mut self, index : usize, dist : f32) -> f32 {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_abs(dist, vels[index])
            }

            #[inline]
            fn drive_comp_rel_async(&mut self, index : usize, dist : f32) {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_rel_async(dist, vels[index])
            }

            #[inline]
            fn drive_comp_abs_async(&mut self, index : usize, dist : f32) {
                let vels = *self.max_vels();
                self.comps_mut()[index].drive_abs_async(dist, vels[index])
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
        fn set_endpoint(&mut self, gammas : &Gammas<N>) -> [bool; N] {
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