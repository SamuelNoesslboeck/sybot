use glam::Vec3;

use crate::Robot;

pub trait SafeRobot<const N : usize> : Robot<N>
{
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
}