use glam::Vec3;

use crate::Robot;

pub type Syomat = crate::BasicRobot<3, 1, 0>;

impl Robot<3> for Syomat 
{
    type Error = std::io::Error;

    // Position
        #[inline]
        fn gammas_from_phis(&self, phis : crate::Phis<3>) -> stepper_lib::Gammas<3> {
            phis
        }

        #[inline]
        fn phis_from_gammas(&self, gammas : stepper_lib::Gammas<3>) -> crate::Phis<3> {
            gammas
        }

        #[inline]
        fn deco_axis(&self) -> Vec3 {
            self.get_tool().unwrap().get_vec()
        }
    //

    fn vecs_from_phis(&self, phis : &crate::Phis<3>) -> crate::Vectors<3> {
        todo!()
    }

    fn phis_from_def_vec(&self, pos : Vec3) -> crate::Phis<3> {
        todo!()
    }

    fn reduce_to_def(&self, pos : Vec3, dec_ang : f32) -> Vec3 {
        todo!()
    }

    fn phis_from_vec(&self, pos : Vec3, dec_ang : f32) -> crate::Phis<3> {
        todo!()
    }

    fn inertias_from_vecs(&self, vecs : &crate::Vectors<3>) -> stepper_lib::Inertias<3> {
        todo!()
    }

    fn forces_from_vecs(&self, vecs : &crate::Vectors<3>) -> stepper_lib::Forces<3> {
        todo!()
    }

    fn update(&mut self, phis : Option<&crate::Phis<3>>) {
        todo!()
    }

    fn measure(&mut self, acc : u64) -> Result<(), [bool; 3]> {
        todo!()
    }

    fn measure_async(&mut self, acc : u64) {
        todo!()
    }

    fn set_limit(&mut self) {
        todo!()
    }

    fn get_tool(&self) -> Option<&Box<dyn stepper_lib::Tool + std::marker::Send>> {
        todo!()
    }

    fn set_tool_id(&mut self, tool_id : usize) {
        todo!()
    }
}