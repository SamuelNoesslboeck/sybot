use glam::Vec3;
use syarm_lib::SyArm;

fn main() {
    let syarm_calc = SyArm::load_json("res/syarm_const.json");

    let angles = syarm_calc.get_with_fixed_dec(Vec3::new(50.0, 300.0, 120.0), 0.0); 

    dbg!(
        angles,
        syarm_calc.points_by_angles(&angles)
    );
}