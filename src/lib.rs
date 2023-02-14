#![crate_name = "sybot_lib"]
//! # SyBot Library
//! 
//! Control and calculation library various robots

// Module decleration
    pub mod intpr;
    pub use intpr::init_interpreter;

    pub mod robot;
    pub use robot::*;

    pub mod server;

    mod types;
    pub use types::*;

    #[cfg(test)]
    mod tests;
//

// Imports
use std::vec;
use std::f32::consts::PI;

use stepper_lib::{Component, ComponentGroup};
use stepper_lib::comp::{Tool, Gammas, Omegas, Inertias, Forces};
use stepper_lib::math::{inertia_point, inertia_rod_constr, forces_segment, inertia_to_mass, forces_joint};

// Public imports
pub use stepper_lib::{JsonConfig, MachineConfig};
pub use stepper_lib::gcode::Interpreter;

// Constants
/// Gravitational acceleration as vector
const G : Vec3 = Vec3 { x: 0.0, y: 0.0, z: -9.805 };

/// Calculation and control struct for the SyArm robot
pub struct SyArm
{
    pub conf : Option<JsonConfig>,
    pub mach : MachineConfig<4, 4, 4>,

    pub vars : RobotVars,

    // Controls
    pub comps : [Box<dyn Component>; 4],

    tool_id : usize
}

/// Returns the angle of a vector to the X-Axis viewed from the Z-Axis
fn top_down_angle(point : Vec3) -> f32 {
    Vec3::new(point.x, point.y, 0.0).angle_between(Vec3::X)
}

fn law_of_cosines(a : f32, b : f32, c : f32) -> f32 {
    ((a.powi(2) + b.powi(2) - c.powi(2)) / 2.0 / a / b).acos()
}

impl Robot<4> for SyArm 
{   
    type Error = SyArmError;

    // Conf
        fn from_conf(conf : JsonConfig) -> Result<Self, std::io::Error> {
            let (mach, comps) = conf.get_machine()?;

            Ok(Self { 
                conf: Some(conf), 
                mach: mach,
                comps: comps,

                vars: RobotVars::default(),

                tool_id: 0
            })
        }

        fn json_conf(&self) -> &Option<JsonConfig> {
            &self.conf
        }
    //

    // Data
        fn comp_group(&self) -> &mut dyn ComponentGroup<4> {
            &mut self.comps
        }

        fn get_vars(&self) -> &RobotVars {
            &self.vars
        }

        fn max_vels(&self) -> &Omegas<4> {
            &self.mach.vels
        }

        fn meas_dists(&self) -> &Gammas<4> {
            &self.mach.meas_dist
        }
    //

    // Position
        fn all_gammas(&self) -> Gammas<4> {
            self.comps.get_dist()
        }

        /// Converts gamma into phi angles
        fn phis_to_gammas(&self, phis : Phis<4>) -> Gammas<4> {
            self.mach.convert_angles(phis, true)
        } 

        /// Returns the four main angles used by the calculations (phis)
        fn all_phis(&self) -> Phis<4> {
            self.gammas_to_phis(self.all_gammas())
        }

        /// Converts phi into gamma angles
        fn gammas_to_phis(&self, gammas : Gammas<4>) -> Phis<4> {
            self.mach.convert_angles(gammas, false)
        }

        // Other
            fn deco_axis(&self) -> Vec3 {
                self.mach.dims[3] + self.get_tool().unwrap().get_vec()
            }

            fn home_pos(&self) -> Gammas<4> {
                self.mach.home
            }
        //
    // 

    // Calculation
        // Position
            fn gammas_to_vecs(&self, gammas : Gammas<4>) -> [Vec3; 4] {
                todo!()
            }

            fn phis_to_vecs(&self, phis : Phis<4>) -> [Vec3; 4] {
                todo!()
            }

            fn vec_to_gammas(&self, pos : Vec3) {
                todo!()
            }

            fn vec_to_phis(&self, pos : Vec3) {
                todo!()
            }

            fn vecs_to_points(&self, vecs : Vectors<4>) -> Points<4> {
                todo!()
            }
        
            fn points_to_vecs(&self, points : Vectors<4>) -> Vectors<4> {
                todo!()
            }
        
            fn reduce_to_defined(&self, pos : Vec3) -> Vec3 {
                todo!()
            }
        //

        // Load
            fn inertias(&self, phis : Phis<4>) -> Inertias<4> {
                todo!()
            }
        
            fn forces(&self, phis : Phis<4>) -> Forces<4> {
                todo!()
            }
        //

        fn update(&self) {
            todo!()
        }
    //

    // Write values
        fn apply_load_inertias(&mut self, inertias : &Inertias<4>) {
            todo!()
        }

        fn apply_load_forces(&mut self, forces : &Forces<4>) {
            todo!()
        }

        fn write_gammas(&self, gammas : &Gammas<4>) {
            todo!()
        }
    //

    // Actions
        fn measure(&mut self, acc : u64) {
            todo!()
        }

        fn measure_async(&mut self, acc : u64) {
            todo!()
        }
    //

    // Tools
        fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>> {
            self.mach.tools.get(self.tool_id)
        }

        fn set_tool_id(&mut self, tool_id : usize) {
            self.tool_id = tool_id;
        }
    //
}

impl SyArm
{
    // Angles
        pub fn valid_gammas(&self, gammas : Gammas<4>) -> bool {
            self.comps.valid_dist(gammas)
        }

        pub fn valid_gammas_verb(&self, gammas : Gammas<4>) -> [bool; 4] {
            self.comps.valid_dist_verb(gammas)
        }

        pub fn valid_phis(&self, phis : Phis<4>) -> bool {
            self.valid_gammas(self.phis_to_gammas(phis))
        }
    //

    // Position calculation
        /// Returns the  points by the given  angles
        pub fn points_by_phis(&self, angles : &Phis<4>) -> Points {
            let [ a_b, a_1, a_2, a_3 ] = self.vectors_by_phis(angles);
            [ 
                self.mach.anchor + a_b,
                self.mach.anchor + a_b + a_1,
                self.mach.anchor + a_b + a_1 + a_2,
                self.mach.anchor + a_b + a_1 + a_2 + a_3
            ]
        }

        /// Get the (most relevant, characteristic) vectors of the robot by the  angles
        pub fn vectors_by_phis(&self, angles : &Phis<4>) -> Vectors {
            let mut vecs = vec![];
            let matr = self.mach.get_axes(angles);

            // Create vectors in default position (Pointing along X-Axis) except base
            for i in 0 .. 4 {
                let mut mat_total = matr[i]; 

                for n in 0 .. i {
                    mat_total = matr[i - n - 1] * mat_total;
                }

                vecs.push(mat_total * self.mach.dims[i]);
            }
            
            vecs.try_into().unwrap()
        }

        /// Get the the angles of the robot when moving to the given point with a fixed decoration axis
        pub fn get_with_fixed_dec(&self, point : Vec3, dec_angle : f32) -> Phis<4> {
            // Rotate onto Y-Z plane
            let phi_b = top_down_angle(point) - PI/2.0;
            let rot_point = Mat3::from_rotation_z(-phi_b) * point;

            // Calculate the decoration vector
            let dec = self.dec_axis();
            let dec_rot = Mat3::from_rotation_x(dec_angle) * dec;

            // Triganlge point
            let d_point = rot_point - dec_rot - self.mach.anchor - self.mach.dims[0];
            
            let phi_h1 = law_of_cosines(d_point.length(), self.mach.dims[1].length(), self.mach.dims[2].length());      // Helper angle for phi_1 calc
            let gamma_2_ = law_of_cosines(self.mach.dims[2].length(), self.mach.dims[1].length(), d_point.length());    // Gamma 2 with side angles
            let mut phi_h = Vec3::Y.angle_between(d_point);                                             // Direct angle towards point

            if 0.0 > d_point.z {
                phi_h = -phi_h;
            }

            let phi_1 = phi_h + phi_h1;
            let phi_2 = gamma_2_ - PI;
            let phi_3 = dec_angle - (phi_1 + phi_2);

            [ phi_b, phi_1, phi_2, phi_3 ]     
        }

        pub fn get_with_fixed_dec_s(&self, x : Option<f32>, y : Option<f32>, z : Option<f32>, dec_angle_o : Option<f32>) -> SyArmResult<Phis<4>> {
            let point = Vec3::new(
                x.unwrap_or(self.vars.point.x),
                y.unwrap_or(self.vars.point.y),
                z.unwrap_or(self.vars.point.z)
            );

            let dec_angle = dec_angle_o.unwrap_or(self.vars.dec_angle);
            
            let phis = self.get_with_fixed_dec(point, dec_angle);
            let gammas = self.phis_to_gammas(phis);
        
            if self.valid_gammas(gammas) { 
                Ok(phis)
            } else {
                let valids = self.valid_gammas_verb(gammas);

                Err(SyArmError::new(format!(
                    "Point {} is out of range! (Gammas: {:?}, Dec: {}) (Valids: ({:?}))", 
                        point, gammas, dec_angle, valids).as_str(), ErrType::OutOfRange))
            }
        }

        pub fn stepper_axes(&self, base_angle : f32) -> Axes {
            let rot_x = Mat3::from_rotation_z(base_angle) * Vec3::NEG_X;
            Axes(
                Vec3::Z,
                rot_x, 
                rot_x,
                rot_x
            )
        } 
    //

    // Advanced velocity calculation
        // pub fn actor_vectors(&self, vecs : &Vectors, phis : &Phis) -> Actors {
        //     let Vectors( a_b, a_1, a_2, a_3 ) = *vecs;
        //     let Axes( x_b, x_1, x_2, x_3 ) = self.stepper_axes(phis.0);

        //     let a_23 = a_2 + a_3;
        //     let a_123 = a_1 + a_23;
        //     let a_b123 = a_b + a_123;

        //     Actors(
        //         ( a_b123 ).cross( x_b ),
        //         ( a_123 ).cross( x_1 ),
        //         ( a_23 ).cross( x_2 ),
        //         ( a_3 ).cross( x_3 )
        //     )
        // }

        // pub fn accel_dyn(&self, phis : &Phis, omegas : Vec3) -> Vec3 {
        //     let Gammas( g_b, g_1, g_2, _ ) = self.gammas_for_phis(phis);

        //     Vec3::new(
        //         self.ctrl_base.accel_dyn(omegas.x, g_b),
        //         self.ctrl_a1.accel_dyn(omegas.y, g_1),
        //         self.ctrl_a2.accel_dyn(omegas.z, g_2)
        //     ) 
        // }

        // pub fn omegas_from_vel(&self, vel : Vec3, phis : &Phis) -> Vec3 {
        //     let vecs = self.vectors_by_phis(phis);
        //     let Actors( eta_b, eta_1, eta_2, _ ) = self.actor_vectors(&vecs, phis);
        //     // let Vectors( a_b, a_1, a_2, a_3 ) = vecs;

        //     // let a_23 = a_2 + a_3;
        //     // let a_123 = a_1 + a_23;
        //     // let a_b123 = a_b + a_123;

        //     let eta_m = Mat3 {
        //         x_axis: eta_b,
        //         y_axis: eta_1,
        //         z_axis: eta_2
        //     };
            
        //     // let vel_red = (a_b123.cross(vel) * a_b123.length().powi(-2)).cross(a_b + a_1 + a_2);
            
        //     eta_m.inverse().mul_vec3(vel)
        // }

        // pub fn vel_from_omegas(&self, omegas : Vec3, phis : &Phis) -> Vec3 {
        //     let vecs = self.vectors_by_phis(phis);
        //     let Actors( eta_b, eta_1, eta_2, _ ) = self.actor_vectors(&vecs, phis);

        //     eta_b * omegas.x + eta_1 * omegas.y + eta_2 * omegas.z
        // }
    // 

    // Path generaton
        // pub fn gen_lin_path(&self, pos_0 : Vec3, pos : Vec3, dec_angle : f32, accuracy : f32) -> SyArmResult<SyArmPath> {
        //     let mut path = SyArmPath::new();
        //     let delta_pos = pos - pos_0;

        //     let n_seg = (delta_pos.length() / accuracy).ceil();

        //     for i in 0 .. (n_seg as u64 + 1) {  // +1 for endposition included
        //         let gammas = self.gammas_for_phis(self.get_with_fixed_dec(pos_0 + (i as f32)/n_seg * delta_pos, dec_angle));
        //         if !self.valid_gammas(gammas) {
        //             return Err(SyArmError::new_simple(ErrType::OutOfRange))
        //         }
        //         path.push(gammas)
        //     }

        //     Ok(path)
        // }
    //

    // Load / Inertia calculation
        pub fn get_cylinder_vecs(&self, vecs : &Vectors) -> CylVectors {
            let [ _, a_1, a_2, _ ] = *vecs;

            let base_helper = Mat3::from_rotation_z(-self.comps[0].get_dist()) * Vec3::new(0.0, -100.0, 0.0); // TODO: -100.0 Y-Dist HARDCODED FOR TESTING!!!

            CylVectors(
                (a_1 / 2.0 - base_helper, base_helper),
                (a_1 / 2.0 + a_2 / 2.0, a_1 / 2.0),
                (a_1 / 2.0 + a_2 / 2.0, a_2 / 2.0)
            )
        }

        pub fn get_inertias(&self, vecs : &Vectors) -> Inertias {
            let CylVectors( (c1_dir, c1_pos), (_, _), (c2_dir, c2_pos) ) = self.get_cylinder_vecs(vecs);

            let mut index : usize;
            let mut inertias = vec![ ];
            let mut segments = vec![ ];
            let tool = self.get_tool().unwrap();
            let tool_mass = tool.get_mass();
            let mut point = tool.get_vec();

            for i in 0 .. 4 { 
                index = 3 - i;

                point += vecs[index];
                segments.insert(0, (self.mach.sim[index].mass, vecs[index]) );
                inertias.insert(0, inertia_rod_constr(&segments) + inertia_point(point, tool_mass));
            }

            [ 
                inertias[0].z_axis.length() / 1_000_000.0, 
                inertia_to_mass(inertias[1], c1_pos, c1_dir), 
                inertia_to_mass(inertias[2], c2_pos, c2_dir),
                (Mat3::from_rotation_z(-self.comps[0].get_dist()) * inertias[3]).x_axis.length() / 1_000_000.0
            ]
        }

        pub fn apply_inertias(&mut self, inertias : Inertias) {
            self.comps.apply_load_inertia(inertias);
        }

        pub fn get_forces(&self, vecs : &Vectors) -> Forces {
            let [ _, a_1, a_2, a_3 ] = *vecs;
            let CylVectors( (c1_dir, c1_pos), (_, c2_pos_1), (c2_dir_2, c2_pos_2) ) = self.get_cylinder_vecs(vecs);

            let fg_load = G * self.vars.load;
            let fg_tool = G * self.get_tool().unwrap().get_mass();

            let fgs : [Vec3; 4] = self.mach.sim.iter().map(
                |sim| sim.mass * G
            ).collect::<Vec<Vec3>>().try_into().unwrap();

            let a_load = self.get_tool().unwrap().get_vec() + a_3;

            let (t_3, f_3) = forces_joint(&vec![ (fg_load + fg_tool, a_load), (fgs[3], a_3 / 2.0) ], Vec3::ZERO);
            let (f_c2, f_2) = forces_segment(&vec![ (f_3, a_2), (fgs[2], a_2 / 2.0) ], t_3, c2_pos_2, c2_dir_2);
            let (f_c1, _ ) = forces_segment(&vec![ (f_2, a_1), (f_c2, c2_pos_1), (fgs[1], a_1 / 2.0) ], Vec3::ZERO, c1_pos, c1_dir);

            [ 0.0, f_c1.length(), f_c2.length(), t_3.length() / 1_000.0 ]
        }

        pub fn apply_forces(&mut self, forces : Forces) {
            self.comps.apply_load_force(forces);
        }
    // 

    // Update
        pub fn update_sim(&mut self) -> Vectors {
            let phis = self.all_phis();
            let vectors = self.vectors_by_phis(&phis);
            let points = self.points_by_phis(&phis);
            
            self.apply_forces(self.get_forces(&vectors));
            self.apply_inertias(self.get_inertias(&vectors));

            self.vars.point = points[3];

            vectors
        }
    //  

    // Control
        pub fn drive_comp_rel(&mut self, index : usize, angle : f32) {
            self.comps[index].drive(angle, self.mach.vels[index]);
        }

        pub fn drive_comp_abs(&mut self, index : usize, angle : f32) {
            self.comps[index].drive_abs(angle, self.mach.vels[index]);
        }

        pub fn measure(&mut self, accuracy : u64) -> Result<(), [bool; 4]> {
            // self.ctrl_base.measure(2*PI, self.cons.omega_b, false);
            let [ _, res_1, res_2, res_3 ] = self.comps.measure(self.mach.meas_dist, self.mach.vels, 
                self.mach.meas.iter().map(|meas| meas.set_val).collect::<Vec<f32>>().try_into().unwrap(), [accuracy; 4]);

            if res_1 & res_2 & res_3 {
                self.set_limit();
                self.update_sim();
                Ok(())
            } else {
                Err([true, res_1, res_2, res_3])
            }
        }

        // Async 
        pub fn drive_comp_rel_async(&mut self, index : usize, angle : f32) {
            self.comps[index].drive_async(angle, self.mach.vels[index]);
        }

        pub fn drive_rel_async(&mut self, angles : Gammas<4>) {
            self.comps.drive_rel_async(angles, self.mach.vels);
        }
        
        pub fn drive_abs_async(&mut self, angles : Gammas<4>) {
            self.comps.drive_async_abs(angles, self.mach.vels);
        }

        pub fn measure_async(&mut self, accuracy : u64) {
            self.comps.measure_async(self.mach.meas_dist, self.mach.vels, [accuracy; 4]);
        }

        pub fn await_inactive(&self) {
            self.comps.await_inactive();
        }

        pub fn set_endpoint(&mut self, gammas : Gammas<4>) {
            self.comps.set_endpoint(gammas);
        }

        pub fn set_limit(&mut self) {
            for i in 0 .. 4 {
                self.comps[i].set_limit(self.mach.limit[i].min, self.mach.limit[i].max);
            }
        }
    // 

    // Debug
        pub fn write_position(&mut self, angles : &Gammas<4>) {
            self.comps.write_dist(angles);
        }
    // 
}