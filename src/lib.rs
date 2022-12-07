#![crate_name = "syarm_lib"]
//! # SyArm library
//! 
//! Control and calculation library for the SyArm robot

// Module decleration
    mod pvec;
    mod types;
    pub mod interpreter;
//

// Imports
use std::{fs, f32::consts::PI};
use serde::{Serialize, Deserialize};

use stepper_lib::{
    ctrl::PwmStepperCtrl, 
    comp::{Cylinder, GearBearing, CylinderTriangle, Tool, NoTool}, 
    data::StepperData, 
    math::{inertia_point, inertia_rod_constr, forces_segment, inertia_to_mass, forces_joint}
};

// Local imports
use pvec::PVec3;
pub use types::*;
pub use interpreter::init_interpreter;

// Constants
const G : Vec3 = Vec3 { x: 0.0, y: 0.0, z: -9.805 };

pub const FORCES_ZERO : Forces = Forces(0.0, 0.0, 0.0, 0.0);
pub const INERTIAS_ZERO : Inertias = Inertias(0.0, 0.0, 0.0, 0.0);

// Structures
    /// All construction constants for the syarm
    #[derive(Serialize, Deserialize, Clone)]
    pub struct Constants 
    {
        // Circuit
        /// Voltage supplied to the motors
        pub u : f32,                    

        /// Direction ctrl pin for the base controller
        pub pin_dir_b : u16,         
        /// Step ctrl pin for the base controller
        pub pin_step_b : u16,          
        pub pin_meas_b : u16,

        /// Direction ctrl pin for the first cylinder
        pub pin_dir_1 : u16, 
        /// Step ctrl pin for the first cylinder
        pub pin_step_1 : u16, 
        pub pin_meas_1 : u16,

        pub pin_dir_2 : u16,
        pub pin_step_2 : u16,
        pub pin_meas_2 : u16,

        pub pin_dir_3 : u16,
        pub pin_step_3 : u16,
        pub pin_meas_3 : u16,

        // Measured
        pub meas_b : f32,
        pub meas_a1 : f32,
        pub meas_a2 : f32,
        pub meas_a3 : f32,

        // Construction
        pub a_b : PVec3,

        /// Length of the first arm segment
        pub l_a1 : f32,
        /// Length of the second arm segment
        pub l_a2 : f32,
        /// Length of the third arm segment
        pub l_a3 : f32,

        pub l_c1a : f32,
        pub l_c1b : f32, 
        pub l_c2a : f32,
        pub l_c2b : f32,

        pub delta_1a : f32,
        pub delta_1b : f32,
        pub delta_2a : f32,
        pub delta_2b : f32,

        pub phib_min : f32,
        pub phib_max : f32,
        pub omega_b : f32,
        pub ratio_b : f32,

        pub c1_max : f32,
        pub c1_v : f32,
        pub ratio_1 : f32,

        pub c2_max : f32,
        pub c2_v : f32,
        pub ratio_2 : f32,

        pub phi3_max : f32,
        pub omega_3 : f32,
        pub ratio_3 : f32,

        // Load calculation
        pub m_b : f32,
        pub m_a1 : f32,
        pub m_a2 : f32,
        pub m_a3 : f32
    }

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
        pub cons : Constants,
        pub vars : Variables,
        pub tool : Box<dyn Tool>,

        // Controls
        pub ctrl_base : GearBearing,
        pub ctrl_a1 : CylinderTriangle,
        pub ctrl_a2 : CylinderTriangle,
        pub ctrl_a3 : GearBearing
    }
//

/// Returns the angle of a vector to the X-Axis viewed from the Z-Axis
pub fn top_down_angle(point : Vec3) -> f32 {
    Vec3::new(point.x, point.y, 0.0).angle_between(Vec3::X)
}

fn _angle_to_deg(angles : Phis) -> Phis {
    Phis( 
        angles.0 * 180.0 / PI,
        angles.1 * 180.0 / PI,
        angles.2 * 180.0 / PI,
        angles.3 * 180.0 / PI
    )
}

pub fn law_of_cosines(a : f32, b : f32, c : f32) -> f32 {
    ((a.powi(2) + b.powi(2) - c.powi(2)) / 2.0 / a / b).acos()
}

impl SyArm
{
    // IO
        /// Creates a new syarm instance by a constants table
        pub fn from_const(cons : Constants) -> Self {
            Self { 
                tool: Box::new(NoTool::new()),    
                ctrl_base: GearBearing { 
                    ctrl: Box::new(PwmStepperCtrl::new(
                        StepperData::mot_17he15_1504s(cons.u), cons.pin_dir_b, cons.pin_step_b
                    )), 
                    ratio: cons.ratio_b
                }, 
                ctrl_a1: CylinderTriangle::new(
                    Cylinder { 
                        ctrl: Box::new(PwmStepperCtrl::new(
                            StepperData::mot_17he15_1504s(cons.u), cons.pin_dir_1, cons.pin_step_1
                        )), 
                        rte_ratio: cons.ratio_1
                    },
                    cons.l_c1a, 
                    cons.l_c1b
                ), 
                ctrl_a2: CylinderTriangle::new(
                    Cylinder { 
                        ctrl: Box::new(PwmStepperCtrl::new(
                            StepperData::mot_17he15_1504s(cons.u), cons.pin_dir_2, cons.pin_step_2
                        )), 
                        rte_ratio: cons.ratio_2,
                    },
                    cons.l_c2a,
                    cons.l_c2b
                ), 
                ctrl_a3: GearBearing { 
                    ctrl: Box::new(PwmStepperCtrl::new(
                        StepperData::mot_17he15_1504s(cons.u), cons.pin_dir_3, cons.pin_step_3
                    )), 
                    ratio: cons.ratio_3
                }, 
                cons, 
                vars: Variables {
                    load: 0.0,

                    dec_angle: 0.0,
                    point: Vec3::ZERO
                }
            }
        }  

        /// Loads a new SyArm instance by creating a constants table out of the json file content at the given path
        pub fn load_json(path : &str) -> Self {
            let json_content  = fs::read_to_string(path).unwrap();
            return Self::from_const(serde_json::from_str(json_content.as_str()).unwrap());
        }

        // pub fn load_var(&mut self, path : &str) {
            
        // }

        // pub fn save_var(&self, path : &str) {
            
        // }

        /// Initializes measurement systems
        pub fn init_meas(&mut self) {
            self.ctrl_base.ctrl.init_meas(self.cons.pin_meas_b);
            self.ctrl_a1.cylinder.ctrl.init_meas(self.cons.pin_meas_1);
            self.ctrl_a2.cylinder.ctrl.init_meas(self.cons.pin_meas_2);
            self.ctrl_a3.ctrl.init_meas(self.cons.pin_meas_3);
        }
    // 

    // Angles
        // Phi: Used by calculation (rotation matrix)
        // Gamma: Used for controls (motor positioning)

        // Base
            /// Get the angles used by the calculations for the base
            pub fn phi_b(&self, gamma_b : f32) -> f32 {
                gamma_b
            }

            /// Get the angle used by the controls for the base
            pub fn gamma_b(&self, phi_b : f32) -> f32 {
                phi_b
            }
        //
        
        // First arm segment
            /// Get the angles used by the calculations for the first arm segment
            pub fn phi_a1(&self, gamma_a1 : f32) -> f32 {
                PI - gamma_a1 - self.cons.delta_1a - self.cons.delta_1b
            }

            /// Get the angle used by the controls for the first arm segment
            pub fn gamma_a1(&self, phi_a1 : f32) -> f32 {
                PI - phi_a1 - self.cons.delta_1a - self.cons.delta_1b
            }
        //
        
        // Second arm segment
            /// Get the angles used by the calculations for the second arm segment
            pub fn phi_a2(&self, gamma_a2 : f32) -> f32 {
                gamma_a2 + self.cons.delta_2a + self.cons.delta_2b - PI
            }

            /// Get the angle used by the controls for the second arm segment
            pub fn gamma_a2(&self, phi_a2 : f32) -> f32 {
                PI + phi_a2 - self.cons.delta_2a - self.cons.delta_2b
            }
        //
        
        // Third arm segment
            /// Get the angles used by the calculations for the third arm segment
            pub fn phi_a3(&self, gamma_a3 : f32) -> f32 {
                gamma_a3
            }

            /// Get the angle used by the controls for the third arm segment
            pub fn gamma_a3(&self, phi_a3 : f32) -> f32 {
                phi_a3
            }
        //

        /// Returns the four main angles used by the controls (gammas)
        pub fn get_all_gammas(&self) -> Gammas {
            Gammas( self.ctrl_base.get_pos(), self.ctrl_a1.get_gamma(), self.ctrl_a2.get_gamma(), self.ctrl_a3.get_pos() )
        }

        /// Converts gamma into phi angles
        pub fn gammas_for_phis(&self, phis : &Phis) -> Gammas {
            Gammas( self.gamma_b(phis.0), self.gamma_a1(phis.1), self.gamma_a2(phis.2), self.gamma_a3(phis.3) )
        } 

        /// Returns the four main angles used by the calculations (phis)
        pub fn get_all_phis(&self) -> Phis {
            Phis( 
                self.phi_b(self.ctrl_base.get_pos()), 
                self.phi_a1(self.ctrl_a1.get_gamma()), 
                self.phi_a2(self.ctrl_a2.get_gamma()), 
                self.phi_a3(self.ctrl_a3.get_pos())
            )
        }

        /// Converts phi into gamma angles
        pub fn phis_for_gammas(&self, gammas : &Gammas) -> Phis {
            Phis( self.phi_a1(gammas.0), self.phi_a1(gammas.1), self.phi_a2(gammas.2), self.phi_a3(gammas.3) )
        }

        pub fn valid_gammas(&self, gammas : Gammas) -> bool {
            let Gammas( g_b, g_1, g_2, g_3 ) = gammas;

            if (g_b < 0.0) | (g_1 < 0.0) | (g_2 < 0.0) | (g_3 < 0.0) {
                return false;
            }


        }
    //

    // Position calculation
        /// Get the vector of the decoration axis
        pub fn a_dec(&self) -> PVec3 {
            PVec3::new(Vec3::new(0.0, self.cons.l_a3, 0.0) + self.tool.get_vec())
        }

        /// Returns the  points by the given  angles
        pub fn get_points_by_phis(&self, angles : &Phis) -> Points {
            let Vectors(a_b, a_1, a_2, a_3) = self.get_vectors_by_phis(angles);
            Points( 
                a_b,
                a_b + a_1,
                a_b + a_1 + a_2,
                a_b + a_1 + a_2 + a_3
            )
        }

        /// Get the (most relevant, characteristic) vectors of the robot by the  angles
        pub fn get_vectors_by_phis(&self, angles : &Phis) -> Vectors {
            // Rotation matrices used multiple times
            let base_rot = Mat3::from_rotation_z(angles.0);
            let a1_rot = Mat3::from_rotation_x(angles.1);
            let a2_rot = Mat3::from_rotation_x(angles.2);
            let a3_rot = Mat3::from_rotation_x(angles.3);

            // Create vectors in default position (Pointing along X-Axis) except base
            let a_b = self.cons.a_b.v;
            let a_1 = Vec3::new(0.0, self.cons.l_a1,  0.0);
            let a_2 = Vec3::new(0.0, self.cons.l_a2, 0.0);
            let a_3 = self.a_dec().v;

            // Multiply up
            Vectors( 
                base_rot * a_b,
                base_rot * a1_rot * a_1,
                base_rot * a1_rot * a2_rot * a_2,
                base_rot * a1_rot * a2_rot * a3_rot * a_3
            )
        }

        pub fn get_with_fixed_dec_rel(&self, x : Option<f32>, y : Option<f32>, z : Option<f32>, dec_angle_o : Option<f32>) -> Phis {
            let point = Vec3::new(
                x.unwrap_or(self.vars.point.x),
                y.unwrap_or(self.vars.point.y),
                z.unwrap_or(self.vars.point.z)
            );

            let dec_angle = dec_angle_o.unwrap_or(self.vars.dec_angle);
            
            self.get_with_fixed_dec(point, dec_angle)
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

            Phis( phi_b, phi_1, phi_2, phi_3 )         
        }
    //

    // Load / Inertia calculation
        pub fn get_cylinder_vecs(&self, vecs : &Vectors) -> CylVectors {
            let Vectors( _, a_1, a_2, _ ) = *vecs;

            let base_helper = Mat3::from_rotation_z(-self.ctrl_base.get_pos()) * Vec3::new(0.0, -self.cons.l_c1a, 0.0);

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
            let j_3 = inertia_rod_constr(&segments) + inertia_point(a_3 + self.tool.get_vec(), self.tool.get_mass());

            segments.insert(0, (self.cons.m_a2, a_2));
            let j_2 = inertia_rod_constr(&segments) + inertia_point(a_2 + a_3 + self.tool.get_vec(), self.tool.get_mass());

            segments.insert(0, (self.cons.m_a1, a_1));
            let j_1 = inertia_rod_constr(&segments) + inertia_point(a_1 + a_2 + a_3 + self.tool.get_vec(), self.tool.get_mass());

            segments.insert(0, (self.cons.m_b, a_b));
            let j_b = inertia_rod_constr(&segments) + inertia_point(a_b + a_1 + a_2 + a_3 + self.tool.get_vec(), self.tool.get_mass());

            Inertias( 
                j_b.z_axis.length() / 1_000_000.0, 
                inertia_to_mass(j_1, c1_pos, c1_dir), 
                inertia_to_mass(j_2, c2_pos, c2_dir),
                (Mat3::from_rotation_z(-self.ctrl_base.get_pos()) * j_3).x_axis.length() / 1_000_000.0
            )
        }

        pub fn apply_inertias(&mut self, inertias : Inertias) {
            let Inertias( j_b, m_1, m_2, j_3 ) = inertias;

            self.ctrl_base.apply_load_j(j_b);
            self.ctrl_a1.cylinder.apply_load_m(m_1);
            self.ctrl_a2.cylinder.apply_load_m(m_2);
            self.ctrl_a3.apply_load_j(j_3);
        }

        pub fn get_forces(&self, vecs : &Vectors) -> Forces {
            let Vectors( _, a_1, a_2, a_3 ) = *vecs;
            let CylVectors( (c1_dir, c1_pos), (_, c2_pos_1), (c2_dir_2, c2_pos_2) ) = self.get_cylinder_vecs(vecs);

            let fg_load = G * self.vars.load;
            let fg_tool = G * self.tool.get_mass();

            let fg_3 = G * self.cons.m_a3;
            let fg_2 = G * self.cons.m_a2;
            let fg_1 = G * self.cons.m_a1;

            let a_load = self.tool.get_vec() + a_3;

            let (t_3, f_3) = forces_joint(&vec![ (fg_load + fg_tool, a_load), (fg_3, a_3 / 2.0) ], Vec3::ZERO);
            let (f_c2, f_2) = forces_segment(&vec![ (f_3, a_2), (fg_2, a_2 / 2.0) ], t_3, c2_pos_2, c2_dir_2);
            let (f_c1, _ ) = forces_segment(&vec![ (f_2, a_1), (f_c2, c2_pos_1), (fg_1, a_1 / 2.0) ], Vec3::ZERO, c1_pos, c1_dir);

            Forces( 0.0, f_c1.length(), f_c2.length(), t_3.length() / 1_000.0 )
        }

        pub fn apply_forces(&mut self, forces : Forces) {
            let Forces( t_b, f_1, f_2, t_3 ) = forces;

            self.ctrl_base.apply_load_t(t_b);
            self.ctrl_a1.cylinder.apply_load_f(f_1);
            self.ctrl_a2.cylinder.apply_load_f(f_2);
            self.ctrl_a3.apply_load_t(t_3);
        }
    // 

    // Update
        pub fn update_sim(&mut self) {
            let phis = self.get_all_phis();
            let vectors = self.get_vectors_by_phis(&phis);
            let points = self.get_points_by_phis(&phis);
            
            self.apply_forces(self.get_forces(&vectors));
            self.apply_inertias(self.get_inertias(&vectors));

            self.vars.point = points.3;
        }
    //  

    // Control
        pub fn drive_base_rel(&mut self, angle : f32) {
            self.ctrl_base.set_pos(self.ctrl_base.get_pos() + angle, self.cons.omega_b);
        }

        pub fn drive_base_abs(&mut self, angle : f32) {
            self.ctrl_base.set_pos(angle, self.cons.omega_b);
        }

        pub fn drive_a1_rel(&mut self, angle : f32) {
            self.ctrl_a1.set_gamma(self.ctrl_a1.get_gamma() + angle, self.cons.c1_v);
        }

        pub fn drive_a1_abs(&mut self, angle : f32) {
            self.ctrl_a1.set_gamma(angle, self.cons.c1_v);
        }

        pub fn drive_a2_rel(&mut self, angle : f32) {
            self.ctrl_a2.set_gamma(self.ctrl_a2.get_gamma() + angle, self.cons.c2_v);
        }

        pub fn drive_a2_abs(&mut self, angle : f32) {
            self.ctrl_a2.set_gamma(angle, self.cons.c2_v);
        }

        pub fn drive_a3_rel(&mut self, angle : f32) {
            self.ctrl_a3.set_pos(self.ctrl_a3.get_pos() + angle, self.cons.omega_3);
        }

        pub fn drive_a3_abs(&mut self, angle : f32) {
            self.ctrl_a3.set_pos(angle, self.cons.omega_3);
        }

        pub fn drive_to_angles(&mut self, angles : Gammas) {
            let Gammas( g_b, g_1, g_2, g_3 ) = angles;
            
            self.drive_base_abs(g_b);
            self.drive_a1_abs(g_1);
            self.drive_a2_abs(g_2);
            self.drive_a3_abs(g_3);
        }

        pub fn measure(&mut self, accuracy : u64) {
            // self.ctrl_base.measure(2*PI, self.cons.omega_b, false);
            self.ctrl_a1.cylinder.measure(self.cons.l_c1a + self.cons.l_c1b, self.cons.c1_v, false, self.cons.meas_a1, accuracy);
            self.ctrl_a2.cylinder.measure(self.cons.l_c2a + self.cons.l_c2b, self.cons.c2_v, false, self.cons.meas_a2, accuracy);
            self.ctrl_a3.measure(2.0*PI, self.cons.omega_3, false, self.cons.meas_a3, accuracy);
        }
    //

    // Debug
        pub fn debug_pins(&self) {
            self.ctrl_base.ctrl.debug_pins();
            self.ctrl_a1.cylinder.ctrl.debug_pins();
            self.ctrl_a2.cylinder.ctrl.debug_pins();
            self.ctrl_a3.ctrl.debug_pins();
        }
    // 
}