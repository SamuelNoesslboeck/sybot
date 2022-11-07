use glam::Vec3;
use syarm_lib::SyArm;

fn main() {
    let syarm_calc = SyArm::load("res/syarm_const.json");

    dbg!(
        syarm_calc.points_by_angles(&syarm_calc.get_with_fixed_dec(Vec3::new(500.0, 0.0, 50.0), 0.0))
    );
}