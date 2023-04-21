use glam::Vec3;

use stepper_lib::units::*;

use crate::ActRobot;

pub trait SafeRobot<const C : usize> : ActRobot<C>
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

        fn safe_phis(&self, phi_opts : [Option<f32>; C]) -> Result<[Phi; C], Self::Error> {
            let mut phis = self.phis();

            for i in 0 .. C {
                phis[i] = Phi(phi_opts[i].unwrap_or(phis[i].0));
            }

            self.check_phis(phis)
        }

        fn safe_deco(&self, decos : &[Option<f32>]) -> Vec<f32> {
            let mut safe_deco = vec![0.0; decos.len()];

            for i in 0 .. safe_deco.len() {
                safe_deco[i] = decos[i].unwrap_or(self.vars().decos[i]);
            }

            safe_deco
        }

        fn safe_phis_for_vec(&self, pos : Vec3, deco : &[f32]) -> Result<[Phi; C], Self::Error> {
            let phis = self.phis_from_vec(pos, deco)?;
            self.check_phis(phis)
        }
    // 

    // Validation
        fn check_gammas(&self, gammas : [Gamma; C]) -> Result<[Gamma; C], Self::Error>;

        #[inline]
        fn check_phis(&self, phis : [Phi; C]) -> Result<[Phi; C], Self::Error> {
            Ok(self.phis_from_gammas(self.check_gammas(self.gammas_from_phis(phis))?))
        }
    // 
}