use std::f32::consts::PI;
use std::io::Error;
use std::vec;

use glam::{Vec3, Mat3};

use stepper_lib::SyncCompGroup;
use stepper_lib::math::force::{forces_segment, forces_joint};
use stepper_lib::math::inertia::{inertia_point, inertia_rod_constr, inertia_to_mass, };
use stepper_lib::units::*;

use crate::{Robot, Vectors, SafeRobot, ConfRobot};

// Constants
/// Gravitational acceleration as vector
const G : Vec3 = Vec3 { x: 0.0, y: 0.0, z: -9.805 };

/// Calculation and control struct for the SyArm robot
pub type SyArm = crate::BasicRobot<4, 1, 4, 4>;

/// Returns the angle of a vector to the X-Axis viewed from the Z-Axis
fn top_down_angle(point : Vec3) -> f32 {

    if point.x > 0.0 {
        (point.y / point.x).atan()
    } else if point.x < 0.0 {
        (point.y / point.x).atan() + PI
    } else {
        if point.y > 0.0 {
            PI / 2.0
        } else if point.y < 0.0 {
            -PI / 2.0
        } else {
            0.0
        }
    }
}

fn law_of_cosines(a : f32, b : f32, c : f32) -> f32 {
    ((a.powi(2) + b.powi(2) - c.powi(2)) / 2.0 / a / b).acos()
}

/// Exchange tuple type for directional and positional vectors of the robot 
#[derive(Clone, Copy, Debug)]
pub struct CylVectors(
    /// First cylinder for first segment \
    /// ( Direction, Position )
    pub (Vec3, Vec3),    
    /// Second cylinder for first segment \
    /// ( Direction, Position )
    pub (Vec3, Vec3),       
    /// Second cylinder for second segment \
    /// ( Direction, Position )
    pub (Vec3, Vec3)        
);


impl Robot<4, 1, 4, 4> for SyArm 
{   
    // Types
        type Error = stepper_lib::Error;
    // 

    // Position
        /// Converts gamma into phi angles
        #[inline]
        fn gammas_from_phis(&self, phis : [Phi; 4]) -> [Gamma; 4] {
            self.mach.gammas_from_phis(phis)
        } 

        /// Converts phi into gamma angles
        #[inline]
        fn phis_from_gammas(&self, gammas : [Gamma; 4]) -> [Phi; 4] {
            self.mach.phis_from_gammas(gammas)
        }

        // Other
            #[inline]
            fn deco_axis(&self) -> Vec3 {
                self.mach.dims[3] + self.get_tool().unwrap().get_vec()
            }
        //
    // 

    // Calculation
        // Position
            fn vecs_from_phis(&self, phis : &[Phi; 4]) -> [Vec3; 4] {
                let mut vecs = vec![];
                let matr = self.mach.get_axes(&phis);
    
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

            fn phis_from_def_vec(&self, mut pos : Vec3) -> [Phi; 4] {
                let phi_b = top_down_angle(pos) - PI/2.0;

                pos = Mat3::from_rotation_z(-phi_b) * pos;

                let phi_h1 = law_of_cosines(pos.length(), self.mach.dims[1].length(), self.mach.dims[2].length());      // Helper angle for phi_1 calc
                let gamma_2_ = law_of_cosines(self.mach.dims[2].length(), self.mach.dims[1].length(), pos.length());    // Gamma 2 with side angles
                let mut phi_h = Vec3::Y.angle_between(pos);                                             // Direct angle towards point

                if 0.0 > pos.z {
                    phi_h = -phi_h;
                }

                let phi_1 = phi_h + phi_h1;
                let phi_2 = gamma_2_ - PI;
                // let phi_3 = dec_angle - (phi_1 + phi_2);

                [ Phi(phi_b), Phi(phi_1), Phi(phi_2), Phi::ZERO ]    
            }

            fn reduce_to_def(&self, pos : Vec3, dec_ang : [f32; 1]) -> Vec3 {
                // Rotate onto Y-Z plane
                let phi_b = top_down_angle(pos) - PI/2.0;

                // Calculate the decoration vector
                let dec = self.deco_axis();
                let dec_rot = Mat3::from_rotation_z(phi_b) * Mat3::from_rotation_x(dec_ang[0]) * dec;

                // Triganlge point
                pos - dec_rot - self.mach.anchor - self.mach.dims[0]
            }

            fn phis_from_vec(&self, pos : Vec3, dec_ang : [f32; 1]) -> [Phi; 4] {
                let pos_def = self.reduce_to_def(pos, dec_ang);
                let mut phis = self.phis_from_def_vec(pos_def);
                phis[3] = Phi(dec_ang[0] - (phis[1].0 + phis[2].0));
                phis
            }
        //

        // Load
            fn inertias_from_vecs(&self, vecs : &Vectors<4>) -> [Inertia; 4] {
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
                    Inertia(inertias[0].z_axis.length() / 1_000_000.0), 
                    inertia_to_mass(inertias[1], c1_pos, c1_dir), 
                    inertia_to_mass(inertias[2], c2_pos, c2_dir),
                    Inertia((Mat3::from_rotation_z(-self.comps[0].gamma().0) * inertias[3]).x_axis.length() / 1_000_000.0) // TODO: Get angle from phis
                ]
            }

            fn forces_from_vecs(&self, vecs : &Vectors<4>) -> [Force; 4] {
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

                [ Force::ZERO, Force(f_c1.length()), Force(f_c2.length()), Force(t_3.length() / 1_000.0) ]
            }
        //

        fn update(&mut self, phis_opt : Option<&[Phi; 4]>) -> Result<(), crate::Error> {
            let all_phis = self.phis();

            let phis = match phis_opt {
                Some(phis) => {
                    self.write_phis(phis);
                    phis
                },
                None => &all_phis
            };

            let vectors = self.vecs_from_phis(phis);
            let points = self.points_from_phis(phis);
            
            self.apply_forces(&self.forces_from_vecs(&vectors));
            self.apply_inertias(&self.inertias_from_vecs(&vectors));

            self.vars.point = points[3];
            self.vars.decos = [ phis[1].0 + phis[2].0 + phis[3].0 ];

            for rem in self.remotes() {
                rem.pub_phis(phis)?;
            }

            Ok(())
        }
    //

    // Actions
        fn measure(&mut self) -> Result<[Delta; 4], crate::Error> {
            let deltas = self.comps.measure(self.mach.meas_dist, self.mach.vels, 
                self.mach.meas.iter().map(|meas| meas.set_val).collect::<Vec<Gamma>>().try_into().unwrap())?;

            self.set_limit();
            self.update(None)?;
            Ok(deltas)
        }

        // fn measure_async(&mut self, acc : u64) {
        //     self.comps.measure_async(self.mach.meas_dist, self.mach.vels, 
        //         self.mach.meas.iter().map(|meas| meas.set_val).collect::<Vec<Gamma>>().try_into().unwrap(), [acc; 4]);
        // }

        fn set_end(&mut self, gammas : &[Gamma; 4]) {
            for i in 1 .. 4 {
                self.comps[i].set_end(gammas[i]);
            }
        }
    //
}

impl SyArm
{
    // Load / Inertia calculation
        pub fn get_cylinder_vecs(&self, vecs : &Vectors<4>) -> CylVectors {
            let [ _, a_1, a_2, _ ] = *vecs;

            let base_helper = Mat3::from_rotation_z(-self.comps[0].gamma().0) * Vec3::new(0.0, -100.0, 0.0); // TODO: -100.0 Y-Dist HARDCODED FOR TESTING!!!
            // TODO: Add Gammas for ------------------------------------| 

            CylVectors(
                (a_1 / 2.0 - base_helper, base_helper),
                (a_1 / 2.0 + a_2 / 2.0, a_1 / 2.0),
                (a_1 / 2.0 + a_2 / 2.0, a_2 / 2.0)
            )
        }
}

impl SafeRobot<4, 1, 4, 4> for SyArm {
    fn check_gammas(&self, gammas : [Gamma; 4]) -> Result<[Gamma; 4], ([Delta; 4], Self::Error)> {
        let valids = self.comps.lims_for_gammas(&gammas);
        
        for valid in valids {
            if valid.is_normal() {
                return Err((valids, Error::new(std::io::ErrorKind::InvalidInput, 
                    format!("The Gammas/Phis given are not valid {:?}", valids))))
            }
        }

        Ok(gammas)
    }
}