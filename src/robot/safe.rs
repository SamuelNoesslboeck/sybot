use glam::Vec3;

use stepper_lib::units::*;

use crate::ActRobot;

pub trait SafeRobot<const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> : ActRobot<COMP, DECO, DIM, ROT>
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

        fn safe_phis(&self, phi_opts : [Option<f32>; COMP]) -> Result<[Phi; COMP], ([Delta; COMP], Self::Error)> {
            let mut phis = self.phis();

            for i in 0 .. COMP {
                phis[i] = Phi(phi_opts[i].unwrap_or(phis[i].0));
            }

            self.check_phis(phis)
        }

        fn safe_deco(&self, decos : [Option<f32>; DECO]) -> [f32; DECO] {
            let mut safe_deco = [0.0; DECO];

            for i in 0 .. DECO {
                safe_deco[i] = decos[i].unwrap_or(self.vars().decos[i]);
            }

            safe_deco
        }

        fn safe_phis_for_vec(&self, pos : Vec3, deco : [f32; DECO]) -> Result<[Phi; COMP], ([Delta; COMP], Self::Error)> {
            let phis = self.phis_from_vec(pos, deco);
            self.check_phis(phis)
        }
    // 

    // Validation
        fn check_gammas(&self, gammas : [Gamma; COMP]) -> Result<[Gamma; COMP], ([Delta; COMP], Self::Error)>;

        #[inline]
        fn check_phis(&self, phis : [Phi; COMP]) -> Result<[Phi; COMP], ([Delta; COMP], Self::Error)> {
            Ok(self.phis_from_gammas(self.check_gammas(self.gammas_from_phis(phis))?))
        }
    // 
}