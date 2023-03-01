use glam::Vec3;
use stepper_lib::{Phi, Gamma};

use crate::Robot;

pub trait SafeRobot<const N : usize, const DECO : usize, const DIM : usize, const ROT : usize> : Robot<N, DECO, DIM, ROT>
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

        fn safe_deco(&self, decos : [Option<f32>; DECO]) -> [f32; DECO] {
            let mut safe_deco = [0.0; DECO];

            for i in 0 .. DECO {
                safe_deco[i] = decos[i].unwrap_or(self.vars().decos[i]);
            }

            safe_deco
        }

        fn safe_phis_for_vec(&self, pos : Vec3, deco : [f32; DECO]) -> Result<[Phi; N], ([bool; N], Self::Error)> {
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