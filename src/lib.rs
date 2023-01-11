#![crate_name = "syarm_lib"]
//! # SyArm library
//! 
//! Control and calculation library for the SyArm robot

// Module decleration
    mod pvec;
    mod types;
    #[cfg(test)]
    mod tests;
    pub mod interpreter;
//

// Imports
use std::{fs, f32::consts::PI, vec};
use serde::{Serialize, Deserialize};

use stepper_lib::{
    Component, ComponentGroup, StepperCtrl, 
    comp::{Cylinder, GearBearing, CylinderTriangle, Tool, NoTool, PencilTool}, 
    data::StepperData, 
    math::{inertia_point, inertia_rod_constr, forces_segment, inertia_to_mass, forces_joint}, 
    paths::{pathphi_new, pathphi_push, PathPhi, StepperPath, path_correction}
};

// Local imports
use pvec::PVec3;
pub use types::*;
pub use interpreter::init_interpreter;
pub use stepper_lib::gcode::Interpreter;

// Constants
/// Gravitational acceleration as vector
const G : Vec3 = Vec3 { x: 0.0, y: 0.0, z: -9.805 };

// Structures
    /// ### Constants
    /// All the constats required for the calculation of the syarm \
    /// JSON I/O is enabled via the `serde_json` library
    #[derive(Serialize, Deserialize)]
    pub struct Constants 
    {
        // Circuit
        /// Voltage supplied to the motors in Volts
        pub u : f32,                    

        /// Direction ctrl pin for the base controller
        pub pin_dir_b : u16,         
        /// Step ctrl pin for the base controller
        pub pin_step_b : u16,          
        /// Measure pin for the base controller
        pub pin_meas_b : u16,

        /// Direction ctrl pin for the first cylinder
        pub pin_dir_1 : u16, 
        /// Step ctrl pin for the first cylinder
        pub pin_step_1 : u16, 
        /// Measure pin for the base controller
        pub pin_meas_1 : u16,

        /// Direction ctrl pin for the second cylinder
        pub pin_dir_2 : u16,
        /// Step ctrl pin for the second cylinder
        pub pin_step_2 : u16,
        /// Measure pin for the second cylinder
        pub pin_meas_2 : u16,

        /// Direction ctrl pin for the third cylinder
        pub pin_dir_3 : u16,
        /// Step ctrl pin for the second cylinder
        pub pin_step_3 : u16,
        /// Measure pin for the third cylinder
        pub pin_meas_3 : u16,

        // Measured
        /// Set value for the base joint when measured in radians
        pub meas_b : f32,
        /// Set value for the first cylinder when measured in mm
        pub meas_a1 : f32,
        /// Set value for the second cylinder when measured in mm
        pub meas_a2 : f32,
        /// Set value for the third joint when measured in radians
        pub meas_a3 : f32,

        // Construction
        /// Base vector when in base position, x, y and z lengths in mm
        pub a_b : PVec3,

        /// Length of the first arm segment in mm
        pub l_a1 : f32,
        /// Length of the second arm segment in mm
        pub l_a2 : f32,
        /// Length of the third arm segment in mm
        pub l_a3 : f32,

        /// Length of a-segment of first cylinder triangle in mm
        pub l_c1a : f32,
        /// Length of b-segment of first cylinder triangle in mm
        pub l_c1b : f32, 
        /// Length of a-segment of second cylinder triangle in mm
        pub l_c2a : f32,
        /// Length of b-segment of second cylinder triangle in mm
        pub l_c2b : f32,

        /// Additional angle of a-segment of first cylinder triangle in mm
        pub delta_1a : f32,
        /// Additional angle of b-segment of first cylinder triangle in mm
        pub delta_1b : f32,
        /// Additional angle of a-segment of second cylinder triangle in mm
        pub delta_2a : f32,
        /// Additional angle of b-segment of second cylinder triangle in mm
        pub delta_2b : f32,

        /// Minimum base joint angle in radians
        pub phib_min : f32, 
        /// Maximum base joint angle in radians
        pub phib_max : f32,
        /// Gear ratio of base joint
        pub ratio_b : f32,

        /// Maximum extension of first cylinder in mm
        pub c1_max : f32,
        /// Spindle pitch in mm per radians
        pub ratio_1 : f32,

        /// Maximum extension of second cylinder in mm
        pub c2_max : f32,
        /// Spindle pitch in mm per radians
        pub ratio_2 : f32,

        /// Maximum base join angle in radians
        pub phi3_max : f32,
        /// Gear ratio of third joint
        pub ratio_3 : f32,

        /// Maximum angluar speeds for the bearings
        pub velocities : [f32; 4],

        // Load calculation
        /// Mass of base in kg
        pub m_b : f32,
        /// Mass of first arm segment in kg
        pub m_a1 : f32,
        /// Mass of second arm segment in kg
        pub m_a2 : f32,
        /// Mass of thrid arm segment in kg
        pub m_a3 : f32,

        /// Safety factor for calculations
        pub sf : f32
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
        pub tools : Vec<Box<dyn Tool + std::marker::Send>>,

        // Controls
        pub ctrls : [Box<dyn Component>; 4],

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
        pub fn from_const(cons : Constants) -> Self {
            Self { 
                tools: vec![ 
                    Box::new(NoTool::new()),
                    Box::new(PencilTool::new(127.0, 0.25))
                ],    
                ctrls: [ 
                    Box::new(GearBearing { 
                        ctrl: StepperCtrl::new(
                            StepperData::mot_17he15_1504s(cons.u, cons.sf), cons.pin_dir_b, cons.pin_step_b
                        ), 
                        ratio: cons.ratio_b
                    }), 
                    Box::new(CylinderTriangle::new(
                        Cylinder { 
                            ctrl: StepperCtrl::new(
                                StepperData::mot_17he15_1504s(cons.u, cons.sf), cons.pin_dir_1, cons.pin_step_1
                            ), 
                            rte_ratio: cons.ratio_1
                        },
                        cons.l_c1a, 
                        cons.l_c1b
                    )), 
                    Box::new(CylinderTriangle::new(
                        Cylinder { 
                            ctrl: StepperCtrl::new(
                                StepperData::mot_17he15_1504s(cons.u, cons.sf), cons.pin_dir_2, cons.pin_step_2
                            ), 
                            rte_ratio: cons.ratio_2,
                        },
                        cons.l_c2a,
                        cons.l_c2b
                    )), 
                    Box::new(GearBearing { 
                        ctrl: StepperCtrl::new(
                            StepperData::mot_17he15_1504s(cons.u, cons.sf), cons.pin_dir_3, cons.pin_step_3
                        ), 
                        ratio: cons.ratio_3
                    }),
                ], 

                cons, 
                vars: Variables {
                    load: 0.0,

                    dec_angle: 0.0,
                    point: Vec3::ZERO
                },

                tool_id: 0
            }
        }
        
        pub fn get_cons_str(&self) -> String {
            serde_json::to_string(&self.cons).unwrap()
        }

        pub fn get_cons_str_pretty(&self) -> String {
            serde_json::to_string_pretty(&self.cons).unwrap()
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
            // TODO do at init
            self.ctrl_base.ctrl.init_meas(self.cons.pin_meas_b);
            self.ctrl_a1.cylinder.ctrl.init_meas(self.cons.pin_meas_1);
            self.ctrl_a2.cylinder.ctrl.init_meas(self.cons.pin_meas_2);
            self.ctrl_a3.ctrl.init_meas(self.cons.pin_meas_3);
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
        pub fn all_gammas(&self) -> Gammas {
            self.ctrls.get_dist()
        }

        /// Converts gamma into phi angles
        pub fn gammas_for_phis(&self, phis : Phis) -> Gammas {
            let [ p_b, p_1, p_2, p_3 ] = phis;
            [ self.gamma_b(p_b), self.gamma_a1(p_1), self.gamma_a2(p_2), self.gamma_a3(p_3) ]
        } 

        /// Returns the four main angles used by the calculations (phis)
        pub fn all_phis(&self) -> Phis {
            let [ g_b, g_1, g_2, g_3 ] = self.all_gammas();
            [ self.phi_b(g_b), self.phi_a1(g_1), self.phi_a2(g_2), self.phi_a3(g_3) ]
        }

        /// Converts phi into gamma angles
        pub fn phis_for_gammas(&self, gammas : Gammas) -> Phis {
            let [ g_b, g_1, g_2, g_3 ] = gammas;
            [ self.phi_b(g_b), self.phi_a1(g_1), self.phi_a2(g_2), self.phi_a3(g_3) ]
        }

        pub fn valid_gammas(&self, gammas : Gammas) -> bool {
            self.ctrls.valid_dist(gammas)
        }

        pub fn valid_gammas_verb(&self, gammas : Gammas) -> [bool; 4] {
            self.ctrls.valid_dist_verb(gammas)
        }

        pub fn valid_phis(&self, phis : Phis) -> bool {
            self.valid_gammas(self.gammas_for_phis(phis))
        }
    //

    // Position calculation
        /// Get the vector of the decoration axis
        pub fn a_dec(&self) -> PVec3 {
            PVec3::new(Vec3::new(0.0, self.cons.l_a3, 0.0) + self.get_tool().unwrap().get_vec())
        }

        /// Returns the  points by the given  angles
        pub fn points_by_phis(&self, angles : &Phis) -> Points {
            let Vectors(a_b, a_1, a_2, a_3) = self.vectors_by_phis(angles);
            Points( 
                a_b,
                a_b + a_1,
                a_b + a_1 + a_2,
                a_b + a_1 + a_2 + a_3
            )
        }

        /// Get the (most relevant, characteristic) vectors of the robot by the  angles
        pub fn vectors_by_phis(&self, angles : &Phis) -> Vectors {
            // Rotation matrices used multiple times
            let base_rot = Mat3::from_rotation_z(angles[0]);
            let a1_rot = Mat3::from_rotation_x(angles[1]);
            let a2_rot = Mat3::from_rotation_x(angles[2]);
            let a3_rot = Mat3::from_rotation_x(angles[3]);

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

        pub fn calc_drive_paths(&self, path : &SyArmPath, vel_max : f32, dist : f32) -> (PathPhi, PathPhi, PathPhi, PathPhi) {
            let mut path_b = pathphi_new();
            let mut path_1 = pathphi_new();
            let mut path_2 = pathphi_new();
            let mut path_3 = pathphi_new();

            let dt = dist / vel_max / path.len() as f32;

            for elem in path {
                let [ g_b, g_1, g_2, g_3 ] = *elem;
                pathphi_push(&mut path_b, (dt, g_b));
                pathphi_push(&mut path_1, (dt, g_1));
                pathphi_push(&mut path_2, (dt, g_2)); 
                pathphi_push(&mut path_3, (dt, g_3));
            }

            ( path_b, path_1, path_2, path_3 )
        }

        // pub fn run_path_correction(&mut self, paths : (PathPhi, PathPhi, PathPhi, PathPhi)) -> Vec<StepperPath> {
        //     let comps_raw : [&mut dyn Component; 4] = [
        //         &mut self.ctrls[0],
        //         &mut self.ctrl_a1,
        //         &mut self.ctrl_a2,
        //         &mut self.ctrl_a3
        //     ];
        //     let mut comps = Vec::from(comps_raw);
        //     let node_count = paths.0.0.len();

        //     path_correction(&mut vec![ paths.0, paths.1, paths.2, paths.3 ], &mut comps, node_count)
        // }
    //

    // Load / Inertia calculation
        pub fn get_cylinder_vecs(&self, vecs : &Vectors) -> CylVectors {
            let Vectors( _, a_1, a_2, _ ) = *vecs;

            let base_helper = Mat3::from_rotation_z(-self.ctrls[0].get_dist()) * Vec3::new(0.0, -self.cons.l_c1a, 0.0);

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
                (Mat3::from_rotation_z(-self.ctrls[0].get_dist()) * j_3).x_axis.length() / 1_000_000.0
            ]
        }

        pub fn apply_inertias(&mut self, inertias : Inertias) {
            self.ctrls.apply_load_inertia(inertias);
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
            self.ctrls.apply_load_force(forces);
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
        pub fn drive_comp_rel(&mut self, index : usize, angle : f32) {
            self.ctrls[index].drive(angle, self.cons.velocities[index]);
        }

        pub fn drive_rel(&mut self, angles : Gammas) {
            self.ctrls.drive(angles, self.cons.velocities);
        }

        pub fn drive_comp_abs(&mut self, index : usize, angle : f32) {
            self.ctrls[index].drive_abs(angle, self.cons.velocities[index]);
        }

        pub fn drive_abs(&mut self, angles : Gammas) {
            self.ctrls.drive_abs(angles, self.cons.velocities);
        }

        pub fn measure(&mut self, accuracy : u64) -> Result<(), (bool, bool, bool, bool)> {
            // self.ctrl_base.measure(2*PI, self.cons.omega_b, false);
            let a_1 = self.ctrl_a1.cylinder.measure(-(self.cons.l_c1a + self.cons.l_c1b), self.cons.c1_v, self.cons.meas_a1, accuracy);
            let a_2 = self.ctrl_a2.cylinder.measure(-(self.cons.l_c2a + self.cons.l_c2b), self.cons.c2_v, self.cons.meas_a2, accuracy);
            let a_3 = self.ctrl_a3.measure(-2.0*PI, self.cons.omega_3,  self.cons.meas_a3, accuracy);

            if a_1 & a_2 & a_3 {
                self.update_sim();
                Ok(())
            } else {
                Err((true, a_1, a_2, a_3))
            }
        }
    //

    // Async control
        pub fn drive_comp_rel_async(&mut self, index : usize, angle : f32) {
            self.ctrls[index].drive_async(angle, self.cons.velocities[index]);
        }

        pub fn drive_rel_async(&mut self, angles : Gammas) {
            self.ctrls.drive_async(angles, self.cons.velocities);
        }
        
        pub fn drive_abs_async(&mut self, angles : Gammas) {
            self.ctrls.drive_async_abs(angles, self.cons.velocities);
        }

        pub fn measure_async(&mut self, accuracy : u64) {
            self.ctrl_base.measure_async(0.0, 0.0, accuracy);
            self.ctrl_a1.cylinder.measure_async(-(self.cons.l_c1a + self.cons.l_c1b), self.cons.c1_v, accuracy);
            self.ctrl_a2.cylinder.measure_async(-(self.cons.l_c2a + self.cons.l_c2b), self.cons.c2_v, accuracy);
            self.ctrl_a3.measure_async(-2.0*PI, self.cons.omega_3, accuracy);
        }

        pub fn await_inactive(&self) {
            self.ctrls.await_inactive();
        }

        pub fn set_endpoint(&mut self) {
            self.ctrl_base.ctrl.set_endpoint(self.cons.meas_b);
            self.ctrl_a1.cylinder.ctrl.set_endpoint(self.cons.meas_a1);
            self.ctrl_a2.cylinder.ctrl.set_endpoint(self.cons.meas_a2);
            self.ctrl_a3.ctrl.set_endpoint(self.cons.meas_a3);
        }
    // 

    // Debug
        pub fn write_position(&mut self, angles : &Gammas) {
            self.ctrls.write_dist(angles);
        }

        // pub fn debug_pins(&self) {
        //     self.ctrl_base.ctrl.debug_pins();
        //     self.ctrl_a1.cylinder.ctrl.debug_pins();
        //     self.ctrl_a2.cylinder.ctrl.debug_pins();
        //     self.ctrl_a3.ctrl.debug_pins();
        // }
    // 
}