use glam::Vec3;
use syarm_lib::{SyArm, INERTIAS_ZERO, FORCES_ZERO};

use std::time::Instant;

fn main() {
    let mut syarm_calc = SyArm::load_json("res/syarm_const.json");

    let angles = syarm_calc.get_with_fixed_dec(Vec3::new(0.0, 300.0, 120.0), 0.0); 
    let vectors = syarm_calc.get_vectors_by_phis(&angles);

    dbg!(
        angles,
        syarm_calc.get_points_by_phis(&angles)
    );

    const TIMES : u64 = 1000;
    let inst = Instant::now();
    let mut inertias = INERTIAS_ZERO; 
    let mut forces = FORCES_ZERO;
    
    for _ in 0 .. TIMES {
        inertias = syarm_calc.get_inertias(&vectors);
        syarm_calc.apply_inertias(inertias);

        forces = syarm_calc.get_forces(&vectors);
        syarm_calc.apply_forces(forces);
    }

    println!("\nCalculation duration: {} for {} times", inst.elapsed().as_secs_f32(), TIMES);

    println!("Inertias: ");
    dbg!(
        inertias,
        syarm_calc.ctrl_base.ctrl.get_data().j_load,
        syarm_calc.ctrl_a1.cylinder.ctrl.get_data().j_load,
        syarm_calc.ctrl_a2.cylinder.ctrl.get_data().j_load,
        syarm_calc.ctrl_a3.ctrl.get_data().j_load
    );

    println!("Forces: ");
    dbg!(
        forces,
        syarm_calc.ctrl_base.ctrl.get_data().t_load,
        syarm_calc.ctrl_a1.cylinder.ctrl.get_data().t_load,
        syarm_calc.ctrl_a2.cylinder.ctrl.get_data().t_load,
        syarm_calc.ctrl_a3.ctrl.get_data().t_load,
    );
}