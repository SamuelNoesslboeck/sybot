use glam::Vec3;
use syarm_lib::SyArm;

use stepper_lib::ctrl::StepperCtrl;

fn main() {
    let mut syarm_calc = SyArm::load_json("res/syarm_const.json");

    let angles = syarm_calc.get_with_fixed_dec(Vec3::new(0.0, 300.0, 120.0), 0.0); 

    dbg!(
        angles,
        syarm_calc.points_by_angles(&angles)
    );

    let inertias = syarm_calc.get_inertias(&syarm_calc.vectors_by_angles(&angles));
    syarm_calc.apply_inertias(inertias);

    println!("Inertias: ");
    dbg!(
        inertias,
        syarm_calc.ctrl_base.ctrl.get_data().j_load,
        syarm_calc.ctrl_a1.cylinder.ctrl.get_data().j_load,
        syarm_calc.ctrl_a2.cylinder.ctrl.get_data().j_load,
        syarm_calc.ctrl_a3.ctrl.get_data().j_load
    );
}