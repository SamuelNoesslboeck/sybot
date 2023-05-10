use alloc::sync::Arc;

use glam::{Mat3, Vec3};

use stepper_lib::data::LinkedData;
use stepper_lib::comp::Tool;
use stepper_lib::units::*;

mod json;
pub use json::*;

#[derive(Debug)]
pub struct MachineConfig<const C : usize>
{
    pub name: String,

    pub lk : Arc<LinkedData>,

    pub anchor : Vec3,
    pub dims : Vec<Vec3>,
    pub axes : Vec<Vec3>,

    pub tools : Vec<Box<dyn Tool + Send>>,

    pub vels : [Omega; C],
    pub home : [Gamma; C],
    pub meas_dist : [Delta; C],
    
    pub ang : [AngleData; C],
    pub sim : [SimData; C],
    pub meas : [MeasInstance; C],
    pub limit : [LimitDecl; C]
}

impl<const C : usize> Default for MachineConfig<C> 
{
    fn default() -> Self {
        Self {
            name: String::new(),
            lk: Default::default(),

            anchor: Default::default(),
            dims: Default::default(),
            axes: Default::default(), 

            tools: vec![],

            vels: [Omega::ZERO; C],
            home: [Gamma::ZERO; C],
            meas_dist: [Delta::ZERO; C],
            
            ang: [Default::default(); C],
            sim: [Default::default(); C],
            meas: [Default::default(); C],
            limit: [Default::default(); C]
        }
    }
}

impl<const C : usize> MachineConfig<C>
{
    pub fn get_axes<const A : usize>(&self, angles : &[Phi; A]) -> Vec<Mat3> {
        if A > self.axes.len() {
            panic!("Bad number of axes! (Required: {}, Present: {})", A, self.axes.len());
        }

        let mut matr = vec![];

        for i in 0 .. A {
            let axis_vec = Vec3::from(self.axes[i]).normalize();

            matr.push(
                if axis_vec == Vec3::X {
                    Mat3::from_rotation_x(angles[i].0)
                } else if axis_vec == Vec3::Y {
                    Mat3::from_rotation_y(angles[i].0)
                } else if axis_vec == Vec3::Z {
                    Mat3::from_rotation_z(angles[i].0)
                } else {
                    Mat3::ZERO
                }
            );
        }

        matr
    }
}

impl<const C : usize> MachineConfig<C>
{
    pub fn gammas_from_phis(&self, phis : [Phi; C]) -> [Gamma; C] {
        let mut gammas = [Gamma::ZERO; C];

        for i in 0 .. C {
            gammas[i] = if self.ang[i].counter { 
                -phis[i].force_to_gamma() + self.ang[i].offset
            } else { 
                phis[i].force_to_gamma() - self.ang[i].offset
            };
        }

        gammas
    }

    pub fn phis_from_gammas(&self, gammas : [Gamma; C]) -> [Phi; C] {
        let mut phis = [Phi::ZERO; C];

        for i in 0 .. C {
            phis[i] = (if self.ang[i].counter { 
                -gammas[i]
            } else { 
                gammas[i]
            } + self.ang[i].offset).force_to_phi();
        }

        phis
    }
}