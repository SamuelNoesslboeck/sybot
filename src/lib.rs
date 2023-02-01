#![crate_name = "syarm_lib"]
//! # SyArm library
//! 
//! Control and calculation library for the SyArm robot

// Module decleration
    pub mod intpr;

    mod types;
    #[cfg(test)]
    mod tests;
//

// Imports
use std::{fs, f32::consts::PI, ops::Index, vec, marker::PhantomData};
use serde::{Serialize, Deserialize, ser::SerializeTuple, de::Visitor};

use stepper_lib::{
    Component, ComponentGroup, StepperCtrl, 
    comp::{Cylinder, GearBearing, CylinderTriangle, Tool, NoTool, PencilTool}, 
    data::StepperConst, 
    math::{inertia_point, inertia_rod_constr, forces_segment, inertia_to_mass, forces_joint}, JsonConfig
};

// Public imports
pub use types::*;
pub use intpr::init_interpreter;
pub use stepper_lib::gcode::Interpreter;

// Constants
/// Gravitational acceleration as vector
const G : Vec3 = Vec3 { x: 0.0, y: 0.0, z: -9.805 };

// Structures
    pub struct Variables
    {
        pub load : f32,

        pub dec_angle : f32,
        pub point : Vec3
    }

    /// Calculation and control struct for the SyArm robot
    pub struct SyArm
    {
        // Values
        pub conf : JsonConfig,
        pub vars : Variables,
        pub dim : [Vec3; 4],

        // Controls
        pub tools : Vec<Box<dyn Tool + std::marker::Send>>,
        pub comps : [Box<dyn Component>; 4],

        tool_id : usize
    }
//

/// Returns the angle of a vector to the X-Axis viewed from the Z-Axis
fn top_down_angle(point : Vec3) -> f32 {
    Vec3::new(point.x, point.y, 0.0).angle_between(Vec3::X)
}

fn law_of_cosines(a : f32, b : f32, c : f32) -> f32 {
    ((a.powi(2) + b.powi(2) - c.powi(2)) / 2.0 / a / b).acos()
}

impl SyArm
{
    // IO
        /// Creates a new syarm instance by a constants table
        pub fn from_conf(conf : JsonConfig) -> Self {
            Self { 
                comps: conf.get_comps(),
                tools: conf.get_tools(),

                dim: conf.get_dim().try_into().unwrap(),

                conf, 
                vars: Variables {
                    load: 0.0,

                    dec_angle: 0.0,
                    point: Vec3::ZERO
                },

                tool_id: 0
            }
        }
    // 

    // Tools
        pub fn get_tool(&self) -> Option<&Box<dyn Tool + std::marker::Send>> {
            self.tools.get(self.tool_id)
        }

        pub fn set_tool_id(&mut self, tool_id : usize) {
            self.tool_id = tool_id;
        }
    //

    // Angles
        /// Returns the four main angles used by the controls (gammas)
        pub fn all_gammas(&self) -> Gammas {
            self.comps.get_dist()
        }

        /// Converts gamma into phi angles
        pub fn gammas_for_phis(&self, phis : Phis) -> Gammas {
            self.comps.dist_without_offset(phis)
        } 

        /// Returns the four main angles used by the calculations (phis)
        pub fn all_phis(&self) -> Phis {
            self.phis_for_gammas(self.all_gammas())
        }

        /// Converts phi into gamma angles
        pub fn phis_for_gammas(&self, gammas : Gammas) -> Phis {
            self.comps.dist_with_offset(gammas) 
        }

        pub fn valid_gammas(&self, gammas : Gammas) -> bool {
            self.comps.valid_dist(gammas)
        }

        pub fn valid_gammas_verb(&self, gammas : Gammas) -> [bool; 4] {
            self.comps.valid_dist_verb(gammas)
        }

        pub fn valid_phis(&self, phis : Phis) -> bool {
            self.valid_gammas(self.gammas_for_phis(phis))
        }
    //

    // Position calculation
        /// Get the vector of the decoration axis
        pub fn a_dec(&self) -> Vec3 {
            self.dim[3] + self.get_tool().unwrap().get_vec()
        }

        /// Returns the  points by the given  angles
        pub fn points_by_phis(&self, angles : &Phis) -> Points {
            let [ a_b, a_1, a_2, a_3 ]= self.vectors_by_phis(angles);
            [ 
                a_b,
                a_b + a_1,
                a_b + a_1 + a_2,
                a_b + a_1 + a_2 + a_3
            ]
        }

        /// Get the (most relevant, characteristic) vectors of the robot by the  angles
        pub fn vectors_by_phis(&self, angles : &Phis) -> Vectors {
            let mut vecs = vec![];
            let matr = self.conf.get_axes(angles);

            // Create vectors in default position (Pointing along X-Axis) except base
            for i in 0 .. matr.len() {
                let mut mat_total = matr[i]; 
                for n in 0 .. i {
                    mat_total = matr[matr.len() - n - 1] * mat_total;
                }

                vecs.push(mat_total * self.dim[i]);
            }
            
            vecs.try_into().unwrap()
        }

        /// Get the the angles of the robot when moving to the given point with a fixed decoration axis
        pub fn get_with_fixed_dec(&self, point : Vec3, dec_angle : f32) -> Phis {
            // Rotate onto Y-Z plane
            let phi_b = top_down_angle(point) - PI/2.0;
            let rot_point = Mat3::from_rotation_z(-phi_b) * point;

            // Calculate the decoration vector
            let dec = self.a_dec().into_y();
            let dec_rot = Mat3::from_rotation_x(dec_angle) * dec.v;

            // Triganlge point
            let d_point = rot_point - dec_rot - self.cons.a_b.v;
            
            let phi_h1 = law_of_cosines(d_point.length(), self.cons.l_a1, self.cons.l_a2);      // Helper angle for phi_1 calc
            let gamma_2_ = law_of_cosines(self.cons.l_a2, self.cons.l_a1, d_point.length());    // Gamma 2 with side angles
            let mut phi_h = Vec3::Y.angle_between(d_point);                                             // Direct angle towards point

            if 0.0 > d_point.z {
                phi_h = -phi_h;
            }

            let phi_1 = phi_h + phi_h1;
            let phi_2 = gamma_2_ - PI;
            let phi_3 = dec_angle - (phi_1 + phi_2);

            [ phi_b, phi_1, phi_2, phi_3 ]     
        }

        pub fn get_with_fixed_dec_s(&self, x : Option<f32>, y : Option<f32>, z : Option<f32>, dec_angle_o : Option<f32>) -> SyArmResult<Phis> {
            let point = Vec3::new(
                x.unwrap_or(self.vars.point.x),
                y.unwrap_or(self.vars.point.y),
                z.unwrap_or(self.vars.point.z)
            );

            let dec_angle = dec_angle_o.unwrap_or(self.vars.dec_angle);
            
            let phis = self.get_with_fixed_dec(point, dec_angle);
            let gammas = self.gammas_for_phis(phis);
        
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
        pub fn gen_lin_path(&self, pos_0 : Vec3, pos : Vec3, dec_angle : f32, accuracy : f32) -> SyArmResult<SyArmPath> {
            let mut path = SyArmPath::new();
            let delta_pos = pos - pos_0;

            let n_seg = (delta_pos.length() / accuracy).ceil();

            for i in 0 .. (n_seg as u64 + 1) {  // +1 for endposition included
                let gammas = self.gammas_for_phis(self.get_with_fixed_dec(pos_0 + (i as f32)/n_seg * delta_pos, dec_angle));
                if !self.valid_gammas(gammas) {
                    return Err(SyArmError::new_simple(ErrType::OutOfRange))
                }
                path.push(gammas)
            }

            Ok(path)
        }
    //

    // Load / Inertia calculation
        pub fn get_cylinder_vecs(&self, vecs : &Vectors) -> CylVectors {
            let Vectors( _, a_1, a_2, _ ) = *vecs;

            let base_helper = Mat3::from_rotation_z(-self.comps[0].get_dist()) * Vec3::new(0.0, -self.cons.l_c1a, 0.0);

            CylVectors(
                (a_1 / 2.0 - base_helper, base_helper),
                (a_1 / 2.0 + a_2 / 2.0, a_1 / 2.0),
                (a_1 / 2.0 + a_2 / 2.0, a_2 / 2.0)
            )
        }

        pub fn get_inertias(&self, vecs : &Vectors) -> Inertias {
            let Vectors( a_b, a_1, a_2, a_3 ) = *vecs;
            let CylVectors( (c1_dir, c1_pos), (_, _), (c2_dir, c2_pos) ) = self.get_cylinder_vecs(vecs);

            let mut segments = vec![ (self.cons.m_a3, a_3) ];
            let j_3 = inertia_rod_constr(&segments) + inertia_point(a_3 + self.get_tool().unwrap().get_vec(), self.get_tool().unwrap().get_mass());

            segments.insert(0, (self.cons.m_a2, a_2));
            let j_2 = inertia_rod_constr(&segments) + inertia_point(a_2 + a_3 + self.get_tool().unwrap().get_vec(), self.get_tool().unwrap().get_mass());

            segments.insert(0, (self.cons.m_a1, a_1));
            let j_1 = inertia_rod_constr(&segments) + inertia_point(a_1 + a_2 + a_3 + self.get_tool().unwrap().get_vec(), self.get_tool().unwrap().get_mass());

            segments.insert(0, (self.cons.m_b, a_b));
            let j_b = inertia_rod_constr(&segments) + inertia_point(a_b + a_1 + a_2 + a_3 + self.get_tool().unwrap().get_vec(), self.get_tool().unwrap().get_mass());

            [ 
                j_b.z_axis.length() / 1_000_000.0, 
                inertia_to_mass(j_1, c1_pos, c1_dir), 
                inertia_to_mass(j_2, c2_pos, c2_dir),
                (Mat3::from_rotation_z(-self.comps[0].get_dist()) * j_3).x_axis.length() / 1_000_000.0
            ]
        }

        pub fn apply_inertias(&mut self, inertias : Inertias) {
            self.comps.apply_load_inertia(inertias);
        }

        pub fn get_forces(&self, vecs : &Vectors) -> Forces {
            let Vectors( _, a_1, a_2, a_3 ) = *vecs;
            let CylVectors( (c1_dir, c1_pos), (_, c2_pos_1), (c2_dir_2, c2_pos_2) ) = self.get_cylinder_vecs(vecs);

            let fg_load = G * self.vars.load;
            let fg_tool = G * self.get_tool().unwrap().get_mass();

            let fg_3 = G * self.cons.m_a3;
            let fg_2 = G * self.cons.m_a2;
            let fg_1 = G * self.cons.m_a1;

            let a_load = self.get_tool().unwrap().get_vec() + a_3;

            let (t_3, f_3) = forces_joint(&vec![ (fg_load + fg_tool, a_load), (fg_3, a_3 / 2.0) ], Vec3::ZERO);
            let (f_c2, f_2) = forces_segment(&vec![ (f_3, a_2), (fg_2, a_2 / 2.0) ], t_3, c2_pos_2, c2_dir_2);
            let (f_c1, _ ) = forces_segment(&vec![ (f_2, a_1), (f_c2, c2_pos_1), (fg_1, a_1 / 2.0) ], Vec3::ZERO, c1_pos, c1_dir);

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

            self.vars.point = points.3;

            vectors
        }
    //  

    // Control
        pub fn measure_dists(&self) -> [f32; 4] {
            [
                0.0,
                -(self.cons.l_c1a + self.cons.l_c1b),
                -(self.cons.l_c2a + self.cons.l_c2b),
                -2.0*PI
            ]
        }

        pub fn drive_comp_rel(&mut self, index : usize, angle : f32) {
            self.comps[index].drive(angle, self.cons.velocities[index]);
        }

        pub fn drive_rel(&mut self, angles : Gammas) {
            self.comps.drive(angles, self.cons.velocities);
        }

        pub fn drive_comp_abs(&mut self, index : usize, angle : f32) {
            self.comps[index].drive_abs(angle, self.cons.velocities[index]);
        }

        pub fn drive_abs(&mut self, angles : Gammas) {
            self.comps.drive_abs(angles, self.cons.velocities);
        }

        pub fn measure(&mut self, accuracy : u64) -> Result<(), [bool; 4]> {
            // self.ctrl_base.measure(2*PI, self.cons.omega_b, false);
            let [ _, res_1, res_2, res_3 ] = self.comps.measure(self.measure_dists(), self.cons.velocities, self.cons.meas, [accuracy; 4]);

            if res_1 & res_2 & res_3 {
                self.update_sim();
                Ok(())
            } else {
                Err([true, res_1, res_2, res_3])
            }
        }

        // Async 
        pub fn drive_comp_rel_async(&mut self, index : usize, angle : f32) {
            self.comps[index].drive_async(angle, self.cons.velocities[index]);
        }

        pub fn drive_rel_async(&mut self, angles : Gammas) {
            self.comps.drive_async(angles, self.cons.velocities);
        }
        
        pub fn drive_abs_async(&mut self, angles : Gammas) {
            self.comps.drive_async_abs(angles, self.cons.velocities);
        }

        pub fn measure_async(&mut self, accuracy : u64) {
            self.comps.measure_async(self.measure_dists(), self.cons.velocities, [accuracy; 4]);
        }

        pub fn await_inactive(&self) {
            self.comps.await_inactive();
        }

        pub fn set_endpoint(&mut self) {
            self.comps.set_endpoint(self.cons.meas);
        }
    // 

    // Debug
        pub fn write_position(&mut self, angles : &Gammas) {
            self.comps.write_dist(angles);
        }

        // pub fn debug_pins(&self) {
        //     self.ctrl_base.ctrl.debug_pins();
        //     self.ctrl_a1.cylinder.ctrl.debug_pins();
        //     self.ctrl_a2.cylinder.ctrl.debug_pins();
        //     self.ctrl_a3.ctrl.debug_pins();
        // }
    // 
}