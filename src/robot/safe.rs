use glam::Vec3;

use stepper_lib::comp::Gammas;

use crate::{Robot, Phis};

pub trait SafeRobot<const N : usize> : Robot<N>
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

        fn safe_phis_for_vec(&self, pos : Vec3, dec_ang : f32) -> Result<Phis<N>, ([bool; N], Self::Error)> {
            let phis = self.phis_from_vec(pos, dec_ang);
            self.valid_phis(&phis)?;
            Ok(phis)
        }
    // 

    // Validation
        fn valid_gammas(&self, gammas : &Gammas<N>) -> Result<(), ([bool; N], Self::Error)>;

        #[inline]
        fn valid_phis(&self, phis : &Phis<N>) -> Result<(), ([bool; N], Self::Error)> {
            self.valid_gammas(&self.gammas_from_phis(*phis))
        }
    // 
}