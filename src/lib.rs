#![crate_name = "syarm_lib"]
//! # SyArm library
//! 
//! Control and calculation library for the SyArm robot

mod pvec;
pub mod interpreter;

use std::{fs, f32::consts::PI};

use serde::{Serialize, Deserialize};
// use serde_json::{Value, json};

use stepper_lib::{ctrl::PwmStepperCtrl, comp::{Cylinder, GearBearing, CylinderTriangle, Tool, NoTool}, data::StepperData};
use pvec::PVec3;

// Types
// Library Types
pub type Vec3 = glam::Vec3;
pub type Mat3 = glam::Mat3;

/// Angles of all the kinematic axis of the robot  \
/// (Base, A1, A2, A3)
pub type Inertias = (f32, f32, f32, f32);
pub type Gammas = (f32, f32, f32, f32);
pub type Phis = (f32, f32, f32, f32);
pub type Points = (Vec3, Vec3, Vec3, Vec3);
pub type Vectors = (Vec3, Vec3, Vec3, Vec3);

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

        pub c1_min : f32, 
        pub c1_max : f32,
        pub c1_v : f32,
        pub ratio_1 : f32,

        pub c2_min : f32, 
        pub c2_max : f32,
        pub c2_v : f32,
        pub ratio_2 : f32,

        pub phi3_min : f32,
        pub phi3_max : f32,
        pub omega_3 : f32,
        pub ratio_3 : f32,

        // Load calculation
        pub m_b : f32,
        pub m_a1 : f32,
        pub m_a2 : f32,
        pub m_a3 : f32
    }

    #[derive(Deserialize, Serialize, Clone)]
    pub struct Variables
    {
        pub load : f32
    }

    /// Calculation and control struct for the SyArm robot
    pub struct SyArm
    {
        // Values
        pub cons : Constants,
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

pub fn _angle_to_deg(angles : Phis) -> Phis {
    return ( 
        angles.0 * 180.0 / PI,
        angles.1 * 180.0 / PI,
        angles.2 * 180.0 / PI,
        angles.3 * 180.0 / PI
    ); 
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
                        rte_ratio: cons.ratio_1,
                        pos_min: cons.c1_min, 
                        pos_max: cons.c1_max
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
                        pos_min: cons.c2_min, 
                        pos_max: cons.c2_max
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
            /// Get the angles 
            pub fn phi_a1(&self, gamma_a1 : f32) -> f32 {
                PI - gamma_a1 - self.cons.delta_1a - self.cons.delta_1b
            }

            /// Get the angle used by the controls for the first arm segment
            pub fn gamma_a1(&self, phi_a1 : f32) -> f32 {
                PI - phi_a1 - self.cons.delta_1a - self.cons.delta_2b
            }
        //
        
        // Second arm segment
            pub fn phi_a2(&self, gamma_a2 : f32) -> f32 {
                gamma_a2 + self.cons.delta_2a + self.cons.delta_2b - PI
            }

            pub fn gamma_a2(&self, phi_a2 : f32) -> f32 {
                PI + phi_a2 - self.cons.delta_2a - self.cons.delta_2b
            }
        //
        
        // Third arm segment
            pub fn phi_a3(&self, gamma_a3 : f32) -> f32 {
                gamma_a3
            }

            pub fn gamma_a3(&self, phi_a3 : f32) -> f32 {
                phi_a3
            }
        //

        pub fn get_all_gamma(&self) -> Gammas {
            ( self.ctrl_base.get_pos(), self.ctrl_a1.get_gamma(), self.ctrl_a2.get_gamma(), self.ctrl_a3.get_pos() )
        }

        pub fn gammas_for_phis(&self, phis : &Phis) -> Gammas {
            ( self.gamma_b(phis.0), self.gamma_a1(phis.1), self.gamma_a2(phis.2), self.gamma_a3(phis.3) )
        } 

        pub fn get_all_phis(&self) -> Phis {
            ( 
                self.phi_b(self.ctrl_base.get_pos()), 
                self.phi_a1(self.ctrl_a1.get_gamma()), 
                self.phi_a2(self.ctrl_a2.get_gamma()), 
                self.phi_a3(self.ctrl_a3.get_pos())
            )
        }

        pub fn phis_for_gammas(&self, gammas : &Gammas) -> Phis {
            ( self.phi_a1(gammas.0), self.phi_a1(gammas.1), self.phi_a2(gammas.2), self.phi_a3(gammas.3) )
        }
    //

    // Position calculation
        /// Get the vector of the decoration axis
        pub fn a_dec(&self) -> PVec3 {
            PVec3::new(Vec3::new(self.cons.l_a3, 0.0, 0.0) + self.tool.get_vec())
        }

        /// Returns the  points by the given  angles
        pub fn points_by_angles(&self, angles : &Phis) -> Points {
            let (a_b, a_1, a_2, a_3) = self.vectors_by_angles(angles);
            ( 
                a_b,
                a_b + a_1,
                a_b + a_1 + a_2,
                a_b + a_1 + a_2 + a_3
            )
        }

        /// Get the (most relevant, characteristic) vectors of the robot by the  angles
        pub fn vectors_by_angles(&self, angles : &Phis) -> Vectors {
            // Rotation matrices used multiple times
            let base_rot = Mat3::from_rotation_z(angles.0);
            let a1_rot = Mat3::from_rotation_y(angles.1);
            let a2_rot = Mat3::from_rotation_y(angles.2);
            let a3_rot = Mat3::from_rotation_y(angles.3);

            // Create vectors in default position (Pointing along X-Axis) except base
            let a_b = self.cons.a_b.v;
            let a_1 = Vec3::new(self.cons.l_a1, 0.0, 0.0);
            let a_2 = Vec3::new(self.cons.l_a2, 0.0, 0.0);
            let a_3 = self.a_dec().v;

            // Multiply up
            ( 
                base_rot * a_b,
                base_rot * a1_rot * a_1,
                base_rot * a1_rot * a2_rot * a_2,
                base_rot * a1_rot * a2_rot * a3_rot * a_3
            )
        }

        /// Get the the angles of the robot when moving to the given point with a fixed decoration axis
        pub fn get_with_fixed_dec(&self, point : Vec3, dec_angle : f32) -> Gammas {
            // Rotate onto X-Z plane
            let phi_b = top_down_angle(point);
            let rot_point = Mat3::from_rotation_z(-phi_b) * point;

            let dec = self.a_dec().into_x();
            let dec_rot = Mat3::from_rotation_y(dec_angle) * dec.v;

            let d_point = rot_point - dec_rot - Mat3::from_rotation_z(phi_b) * self.cons.a_b.v;
            
            let gamma_1_ = law_of_cosines(d_point.length(), self.cons.l_a1, self.cons.l_a2);
            let gamma_2_ = law_of_cosines(self.cons.l_a2, self.cons.l_a1, d_point.length());
            let mut phi_h = Vec3::X.angle_between(d_point);

            if 0.0 > d_point.z {
                phi_h = -phi_h;
            }

            (
                phi_b,                                          // Base angle
                gamma_1_,                                   // First arm
                gamma_2_ - self.cons.delta_2a - self.cons.delta_2b,              // Second arm
                - (PI - gamma_2) + (phi_h + gamma_1_) - dec_angle     // Third angle
            )         
        }
    //

    // Load / Inertia calculation
        pub fn get_inhertias(&self) {

        }
    // 

    // Control
        pub fn drive_base_rel(&mut self, angle : f32) {
            self.ctrl_base.set_pos(self.ctrl_base.get_pos() + angle, self.cons.omega_b);
        }

        pub fn drive_a1_rel(&mut self, angle : f32) {
            self.ctrl_a1.set_gamma(self.ctrl_a1.get_gamma() + angle, self.cons.c1_v);
        }

        pub fn drive_a2_rel(&mut self, angle : f32) {
            self.ctrl_a2.set_gamma(self.ctrl_a2.get_gamma() + angle, self.cons.c2_v);
        }

        pub fn drive_a3_rel(&mut self, angle : f32) {
            self.ctrl_a3.set_pos(self.ctrl_a3.get_pos() + angle, self.cons.omega_3);
        }

        pub fn drive_to_angles(&mut self, angles : Phis) {
            self.ctrl_base.set_pos(angles.0, self.cons.omega_b);
            self.ctrl_a1.set_gamma(PI - angles.1 - self.cons.delta_1a, self.cons.c1_v);
            self.ctrl_a2.set_gamma(angles.2 - self.cons.delta_2a - self.cons.delta_2b, self.cons.c2_v);
            self.ctrl_a3.set_pos(angles.3, self.cons.omega_3);
        }

        pub fn measure(&mut self, accuracy : u64) {
            // self.ctrl_base.measure(2*PI, self.cons.omega_b, false);
            self.ctrl_a1.cylinder.measure(self.cons.l_c1a + self.cons.l_c1b, self.cons.c1_v, false, 0.0, accuracy);
            self.ctrl_a2.cylinder.measure(self.cons.l_c2a + self.cons.l_c2b, self.cons.c2_v, false, 0.0, accuracy);
            self.ctrl_a3.measure(2.0*PI, self.cons.omega_3, true, 0.0, accuracy);
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