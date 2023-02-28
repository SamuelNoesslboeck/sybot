use glam::Vec3;
use stepper_lib::{Phi, Gamma};

use crate::Robot;

pub trait SafeRobot<const N : usize, const D : usize> : Robot<N, D>
{
    // Correction
        fn safe_pos(&self, x : Option<f32>, y : Option<f32>, z : Option<f32>) -> Vec3 {
            let point = self.vars().point;

            Vec3::new(
                x.unwrap_or(point.x),
                y.unwrap_or(point.y),
                z.unwrap_or(point.z)
            )
        }

        fn safe_deco(&self, deco : Option<f32>) -> f32 {
            deco.unwrap_or(self.vars().dec_angle)
        }

        fn safe_phis_for_vec(&self, pos : Vec3, deco : [f32; D]) -> Result<[Phi; N], ([bool; N], Self::Error)> {
            let phis = self.phis_from_vec(pos, deco);
            self.valid_phis(&phis)?;
            Ok(phis)
        }
    // 

    // Validation
        fn valid_gammas(&self, gammas : &[Gamma; N]) -> Result<(), ([bool; N], Self::Error)>;

        #[inline]
        fn valid_phis(&self, phis : &[Phi; N]) -> Result<(), ([bool; N], Self::Error)> {
            self.valid_gammas(&self.gammas_from_phis(*phis))
        }
    // 
}