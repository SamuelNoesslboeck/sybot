use glam::Vec3;
use stepper_lib::units::*; 

use crate::{ActRobot, Vectors, Robot};

pub type Syomat = crate::BasicRobot<3, 0, 1, 0>;

const G : Alpha = Alpha(-9.805);

#[allow(unused)]
impl ActRobot<3, 0, 1, 0> for Syomat 
{
    type Error = std::io::Error;

    // Position
        #[inline]
        fn gammas_from_phis(&self, phis : [Phi; 3]) -> [Gamma; 3] {
            force_gammas_from_phis(phis)
        }

        #[inline]
        fn phis_from_gammas(&self, gammas : [Gamma; 3]) -> [Phi; 3] {
            force_phis_from_gammas(gammas)
        }

        #[inline]
        fn deco_axis(&self) -> Vec3 {
            self.get_tool().unwrap().get_vec()
        }
    //

    fn vecs_from_phis(&self, phis : &[Phi; 3]) -> Vectors<3> {
        let [ p_x, p_y, p_z ] = phis;
        
        [ Vec3::X * p_x.0, Vec3::Y * p_y.0, Vec3::Z * p_z.0 ]
    }

    fn phis_from_def_vec(&self, pos : Vec3) -> [Phi; 3] {
        [ Phi(pos.x), Phi(pos.y), Phi(pos.z) ]
    }

    #[inline]
    fn reduce_to_def(&self, pos : Vec3, _ : [f32; 0]) -> Vec3 {
        pos
    }

    #[inline]
    fn phis_from_vec(&self, pos : Vec3, deco : [f32; 0]) -> [Phi; 3] {
        self.phis_from_def_vec(self.reduce_to_def(pos, deco))
    }

    fn inertias_from_vecs(&self, _ : &Vectors<3>) -> [Inertia; 3] {
        let tool_inert = match self.get_tool() {
            Some(tool) => Inertia(tool.get_mass()),
            None => Inertia::ZERO
        };

        [
            self.mach.sim[0].mass,
            self.mach.sim[1].mass + tool_inert,
            self.mach.sim[1].mass + self.mach.sim[2].mass + tool_inert,
        ]
    }

    fn forces_from_vecs(&self, vecs : &Vectors<3>) -> [Force; 3] {
        let tool_inert = match self.get_tool() {
            Some(tool) => Inertia(tool.get_mass()),
            None => Inertia::ZERO
        };

        [ 
            Force::ZERO,
            Force::ZERO,
            G * (self.mach.sim[1].mass + self.mach.sim[2].mass + tool_inert)
        ]
    }

    fn update(&mut self, phis_opt : Option<&[Phi; 3]>) -> Result<(), crate::Error> {
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
        self.vars.decos = [ ];

        for rem in self.remotes_mut() {
            rem.pub_phis(phis)?;
        }

        Ok(())
    }

    fn measure(&mut self) -> Result<[Delta; 3], stepper_lib::Error> {
        todo!()
    }
}