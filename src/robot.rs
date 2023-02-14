use glam::Vec3; 

use stepper_lib::{Tool, JsonConfig, ComponentGroup};
use stepper_lib::comp::{Gammas, Omegas, Inertias, Forces};

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

pub trait Robot<const N : usize>
{
    // Type
        type Error : std::error::Error;
    // 

    // Configuration
        /// Creates a new instance of the robot from a Json-Configuration file if it's format is appropriate
        fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error>
            where
                Self: Sized;    

        /// Returns the 
        fn json_conf(&self) -> &Option<JsonConfig>;
    // 

    // Stats and Data
        fn comp_group(&self) -> &mut dyn ComponentGroup<N>;

        fn get_vars(&self) -> &RobotVars;

        fn max_vels(&self) -> &Omegas<N>;
        
        fn meas_dists(&self) -> &Gammas<N>;
    //

    // Position
        /// Returns all the angles used by the controls to represent the components extension/drive distance
        #[inline]
        fn all_gammas(&self) -> Gammas<N> {
            self.comp_group().get_dist()
        }

        /// Converts all angles (by subtracting an offset in most of the cases)
        fn phis_to_gammas(&self, phis : Phis<N>) -> Gammas<N>;

        #[inline]
        fn all_phis(&self) -> Phis<N> {
            self.gammas_to_phis(self.all_gammas())
        }

        fn gammas_to_phis(&self, gammas : Gammas<N>) -> Phis<N>;

        // Other
            fn deco_axis(&self) -> Vec3;
            
            fn anchor(&self) -> Vec3;

            fn home_pos(&self) -> Gammas<N>;
        //
    //

    // Calculation
        // Position
            fn gammas_to_vecs(&self, gammas : Gammas<N>) -> Vectors<N>; 
            
            fn phis_to_vecs(&self, phis : Phis<N>) -> Vectors<N>;

            fn vec_to_gammas(&self, pos : Vec3);

            fn vec_to_phis(&self, pos : Vec3);

            fn vecs_to_points(&self, vecs : &Vectors<N>) -> Points<N> {
                let mut points : Vectors<N> = [Vec3::ZERO; N];
                for i in 0 .. N {
                    points[i] += self.anchor();
                    for n in 0 .. (i + 1) {
                        points[i] += vecs[i];
                    }
                }
                vecs
            }

            fn points_to_vecs(&self, points : &Vectors<N>) -> Vectors<N> {
                let mut vecs : Points<N> = [Vec3::ZERO; N];
                vecs[0] = points[0] - self.anchor();
                for i in 1 .. N {
                    vecs[i] = points[i] - points[i - 1];
                }
                vecs
            }

            fn reduce_to_defined(&self, pos : Vec3) -> Vec3;
        //

        // Correction
            #[inline]
            fn valid_gammas(&self, gammas : &Gammas<N>) -> bool {
                self.comp_group().valid_dist(gammas)
            }  

            #[inline]
            fn valid_phis(&self, phis : &Phis<N>) -> bool {
                self.comp_group().valid_dist(&self.phis_to_gammas(phis))
            }
        //  

        // Load
            fn inertias(&self, phis : &Phis<N>) -> Inertias<N>;

            fn forces(&self, phis : &Phis<N>) -> Forces<N>;
        // 

        fn update(&self, phis : &Phis<N>);
    //

    // Writing values
        #[inline]
        fn apply_load_inertias(&mut self, inertias : &Inertias<N>) {
            self.comp_group().apply_load_inertias(inertias);
        }

        #[inline]
        fn apply_load_forces(&mut self, forces : &Forces<N>) {
            self.comp_group().apply_load_forces(forces);
        }

        #[inline]
        fn write_gammas(&self, gammas : &Gammas<N>) {
            self.comp_group().write_dist(gammas);
        }
    // 

    // Movement
        #[inline]
        fn drive_rel(&mut self, dist : [f32; N]) -> [f32; N] {
            self.comp_group().drive_rel(dist, *self.max_vels())
        }

        #[inline]
        fn drive_abs(&mut self, dist : Gammas<N>) -> [f32; N] {
            self.comp_group().drive_abs(dist, *self.max_vels())
        }

        // Async 
        #[inline]
        fn drive_rel_async(&mut self, angles : [f32; N]) {
            self.comp_group().drive_rel_async(angles, *self.max_vels());
        }
        
        #[inline]
        fn drive_abs_async(&mut self, angles : Gammas<N>) {
            self.comp_group().drive_abs_async(angles, *self.max_vels());
        }

        // Measure
            fn measure(&mut self, acc : u64);

            fn measure_async(&mut self, acc : u64);
        // 

        #[inline]
        fn await_inactive(&self) {
            self.comp_group().await_inactive()
        }

        #[inline]
        fn set_endpoint(&mut self, gammas : &Gammas<N>) {
            self.comp_group().set_endpoint()
        }
    // 

    // Tools
        /// Returns the current tool that is being used by the robot
        fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>>;

        fn set_tool_id(&mut self, tool_id : usize);
    //
}