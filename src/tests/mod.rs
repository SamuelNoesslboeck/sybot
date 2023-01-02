use std::{f32::consts::PI};
use crate::*;

// Test Arm
    #[test]
    fn test_arm() {
        let mut syarm = SyArm::load_json("res/syarm_const.json");

        const ANGLES : Phis = Phis(0.0, PI / 2.0, -PI / 2.0, 0.0);

        syarm.write_position(&syarm.gammas_for_phis(&ANGLES)); 

        syarm.update_sim();

        // let angles = syarm.get_with_fixed_dec(Vec3::new(0.0, 300.0, 120.0), 0.0); 
        // let vectors = syarm.get_vectors_by_phis(&angles);

        // dbg!(
        //     angles,
        //     syarm.get_points_by_phis(&angles)
        // );

        // const TIMES : u64 = 1000;
        // let inst = Instant::now();
        // let mut inertias = INERTIAS_ZERO; 
        // let mut forces = FORCES_ZERO;
        
        // for _ in 0 .. TIMES {
        //     inertias = syarm.get_inertias(&vectors);
        //     syarm.apply_inertias(inertias);

        //     forces = syarm.get_forces(&vectors);
        //     syarm.apply_forces(forces);
        // }

        // println!("\nCalculation duration: {} for {} times", inst.elapsed().as_secs_f32(), TIMES);

        dbg!(syarm.get_inertias(&syarm.get_vectors_by_phis(&ANGLES)));

        println!("Inertias: ");
        dbg!(
            // inertias,
            syarm.ctrl_base.ctrl.driver.lock().unwrap().data.j_load,
            syarm.ctrl_a1.cylinder.ctrl.driver.lock().unwrap().data.j_load,
            syarm.ctrl_a2.cylinder.ctrl.driver.lock().unwrap().data.j_load,
            syarm.ctrl_a3.ctrl.driver.lock().unwrap().data.j_load
        );

        println!("Forces: ");
        dbg!(
            // forces,
            syarm.ctrl_base.ctrl.driver.lock().unwrap().data.t_load,
            syarm.ctrl_a1.cylinder.ctrl.driver.lock().unwrap().data.t_load,
            syarm.ctrl_a2.cylinder.ctrl.driver.lock().unwrap().data.t_load,
            syarm.ctrl_a3.ctrl.driver.lock().unwrap().data.t_load,
        );
    }
// 

// Test move
    #[test]
    fn test_linear_move() {
        let mut syarm = SyArm::load_json("res/syarm_const.json");  

        const ANGLES : Phis = Phis(0.0, PI / 2.0, -PI / 2.0, 0.0);
        const VEL : Vec3 = Vec3::new(50.0, 0.0, 0.0);

        syarm.write_position(&syarm.gammas_for_phis(&ANGLES));
        syarm.update_sim();

        let omega = syarm.omegas_from_vel(VEL, &ANGLES);
        let vecs = syarm.get_vectors_by_phis(&ANGLES);

        println!("Actor vectors:");
        dbg!(syarm.actor_vectors(&vecs, &ANGLES));
        dbg!(syarm.accel_dyn(
            dbg!(omega)
        ));
        dbg!(syarm.vel_from_omegas(omega, &ANGLES));
    }
//