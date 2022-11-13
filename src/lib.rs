#![crate_name = "syarm_lib"]
//! # SyArm library
//! 
//! Control and calculation library for the SyArm robot

mod pvec;
pub mod interpreter;

use std::{fs, f32::consts::PI};

use glam::{Mat3, Vec3};
use serde::{Serialize, Deserialize};
// use serde_json::{Value, json};

use stepper_lib::{controller::{PwmStepperCtrl, Cylinder, GearBearing, CylinderTriangle}, data::StepperData};
use pvec::PVec3;

// Types
pub type MainAngles = (f32, f32, f32, f32);
pub type MainPoints = (Vec3, Vec3, Vec3, Vec3);
pub type MainVectors = (Vec3, Vec3, Vec3, Vec3);

// Structures
    /// All construction constants for the syarm
    #[derive(Serialize, Deserialize, Clone)]
    pub struct Constants 
    {
        // Circuit
        /// Circuit voltage
        pub u : f32,                    

        /// Direction ctrl pin for the base controller
        pub pin_dir_b : u8,         
        /// Step ctrl pin for the base controller
        pub pin_step_b : u8,          

        /// Direction ctrl pin for the first cylinder
        pub pin_dir_1 : u8, 
        /// Step ctrl pin for the first cylinder
        pub pin_step_1 : u8, 

        pub pin_dir_2 : u8,
        pub pin_step_2 : u8,

        pub pin_dir_3 : u8,
        pub pin_step_3 : u8,

        // Construction
        pub a_b : PVec3,

        pub l_a1 : f32,
        pub l_a2 : f32,
        pub l_a3 : f32,

        pub l_c1a : f32,
        pub l_c1b : f32, 
        pub l_c2a : f32,
        pub l_c2b : f32,

        pub delta_1a : f32,
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
        pub ratio_3 : f32
    }

    /// Calculation and control struct for the SyArm robot
    pub struct SyArm
    {
        pub cons : Constants,
        pub tool : Vec3,

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

pub fn main_angle_to_deg(angles : MainAngles) -> MainAngles {
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
        pub fn from_const(cons : Constants) -> Self {
            Self { 
                tool: Vec3::new(0.0, 0.0, 0.0),     // TODO: Add proper tool
                ctrl_base: GearBearing { 
                    ctrl: PwmStepperCtrl::new(
                        StepperData::mot_17he15_1504s(cons.u), cons.pin_dir_b, cons.pin_step_b
                    ), 
                    ratio: cons.ratio_b
                }, 
                ctrl_a1: CylinderTriangle { 
                    l_a: cons.l_c1a, 
                    l_b: cons.l_c1b, 
                    cylinder: Cylinder { 
                        ctrl: PwmStepperCtrl::new(
                            StepperData::mot_17he15_1504s(cons.u), cons.pin_dir_1, cons.pin_step_1
                        ), 
                        rte_ratio: cons.ratio_1,
                        pos_min: cons.c1_min, 
                        pos_max: cons.c1_max
                    }
                }, 
                ctrl_a2: CylinderTriangle { 
                    l_a: cons.l_c2a, 
                    l_b: cons.l_c2b, 
                    cylinder: Cylinder { 
                        ctrl: PwmStepperCtrl::new(
                            StepperData::mot_17he15_1504s(cons.u), cons.pin_dir_2, cons.pin_step_2
                        ), 
                        rte_ratio: cons.ratio_2,
                        pos_min: cons.c2_min, 
                        pos_max: cons.c2_max
                    }
                }, 
                ctrl_a3: GearBearing { 
                    ctrl: PwmStepperCtrl::new(
                        StepperData::mot_17he15_1504s(cons.u), cons.pin_dir_3, cons.pin_step_3
                    ), 
                    ratio: cons.ratio_3
                }, 
                cons, 
            }
        }

        pub fn load(path : &str) -> Self {
            let json_content  = fs::read_to_string(path).unwrap();
            return Self::from_const(serde_json::from_str(json_content.as_str()).unwrap());
        }

        pub fn load_var(&mut self, path : &str) {
            
        }

        pub fn save_var(&self, path : &str) {
            
        }
    // 

    // Angles
        pub fn phi_b(&self) -> f32 {
            self.ctrl_base.get_pos()
        }

        pub fn phi_a1(&self) -> f32 {
            PI - self.ctrl_a1.get_gamma() + self.cons.delta_1a
        }

        pub fn phi_a2(&self) -> f32 {
            self.ctrl_a2.get_gamma() + self.cons.delta_2a + self.cons.delta_2b
        }

        pub fn phi_a3(&self) -> f32 {
            self.ctrl_a3.get_pos()
        }
    //

    // Position calculation
        pub fn a_dec(&self) -> PVec3 {
            PVec3::new(Vec3::new(self.cons.l_a3, 0.0, 0.0) + self.tool)
        }

        pub fn points_by_angles(&self, angles : &MainAngles) -> MainPoints {
            let (a_b, a_1, a_2, a_3) = self.vectors_by_angles(angles);
            ( 
                a_b,
                a_b + a_1,
                a_b + a_1 + a_2,
                a_b + a_1 + a_2 + a_3
            )
        }

        pub fn vectors_by_angles(&self, angles : &MainAngles) -> MainVectors {
            let base_rot = Mat3::from_rotation_z(angles.0);
            let a1_rot = Mat3::from_rotation_y(angles.1);
            let a2_rot = Mat3::from_rotation_y(angles.2);
            let a3_rot = Mat3::from_rotation_y(angles.3);

            let a_b = self.cons.a_b.v;
            let a_1 = Vec3::new(self.cons.l_a1, 0.0, 0.0);
            let a_2 = Vec3::new(self.cons.l_a2, 0.0, 0.0);
            let a_3 = self.a_dec().v;

            ( 
                base_rot * a_b,
                base_rot * a1_rot * a_1,
                base_rot * a1_rot * a2_rot * a_2,
                base_rot * a1_rot * a2_rot * a3_rot * a_3
            )
        }

        /// Get the the angles of the robot when moving to the given point with a fixed decoration axis
        pub fn get_with_fixed_dec(&self, point : Vec3, dec_angle : f32) -> MainAngles {
            let top_angle = top_down_angle(point);
            let r_point = Mat3::from_rotation_z(-top_angle) * point;

            let dec = self.a_dec().into_x();
            let dec_rot = Mat3::from_rotation_y(-dec_angle) * dec.v;

            let d_point = r_point - dec_rot - self.cons.a_b.v;
            
            let gamma_1_ = law_of_cosines(d_point.length(), self.cons.l_a1, self.cons.l_a2);
            let gamma_2 = law_of_cosines(self.cons.l_a2, self.cons.l_a1, d_point.length());
            let mut phi_h = Vec3::X.angle_between(d_point);

            if 0.0 > d_point.z {
                phi_h = -phi_h;
            }

            (
                top_angle,                                          // Base angle
                -(phi_h + gamma_1_),                                   // First arm
                PI - gamma_2,                                            // Second arm
                - (PI - gamma_2) + (phi_h + gamma_1_) - dec_angle     // Third angle
            )         
        }
    //

    // Control
        /// 
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

        pub fn drive_to_angles(&mut self, angles : MainAngles) {
            self.ctrl_base.set_pos(angles.0, self.cons.omega_b);
            self.ctrl_a1.set_gamma(PI - angles.1 - self.cons.delta_1a, self.cons.c1_v);
            self.ctrl_a2.set_gamma(angles.2 - self.cons.delta_2a - self.cons.delta_2b, self.cons.c2_v);
            self.ctrl_a3.set_pos(angles.3, self.cons.omega_3);
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