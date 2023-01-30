use std::{f32::consts::PI, thread::sleep, time::Duration};
use crate::*;

// Test Arm
    #[test]
    fn test_arm() {
        let mut syarm = SyArm::load_json("res/syarm_const.json");

        const ANGLES : Phis = [0.0, PI / 2.0, -PI / 2.0, 0.0];

        syarm.write_position(&syarm.gammas_for_phis(ANGLES)); 

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

        dbg!(syarm.get_inertias(&syarm.vectors_by_phis(&ANGLES)));

        // println!("Inertias: ");
        // dbg!(
        //     // inertias,
        //     syarm.ctrl.ctrl.driver.lock().unwrap().data.j_load,
        //     syarm.ctrl_a1.cylinder.ctrl.driver.lock().unwrap().data.j_load,
        //     syarm.ctrl_a2.cylinder.ctrl.driver.lock().unwrap().data.j_load,
        //     syarm.ctrl_a3.ctrl.driver.lock().unwrap().data.j_load
        // );

        // println!("Forces: ");
        // dbg!(
        //     // forces,
        //     syarm.ctrl_base.ctrl.driver.lock().unwrap().data.t_load,
        //     syarm.ctrl_a1.cylinder.ctrl.driver.lock().unwrap().data.t_load,
        //     syarm.ctrl_a2.cylinder.ctrl.driver.lock().unwrap().data.t_load,
        //     syarm.ctrl_a3.ctrl.driver.lock().unwrap().data.t_load,
        // );
    }
// 

// Test Deg
    #[test]
    fn test_deg() {
        let mut syarm = SyArm::load_json("res/syarm_const.json");
        let dur = Duration::from_secs_f32(0.5);
        let angle = PI / 8.0;
    
        // syarm.debug_pins();
    
        println!("Running movement tests ... ");

        syarm.measure(2).unwrap();
        
        for i in 0 .. 4 {
            print!(" -> Moving base ... ");
            syarm.drive_comp_rel(i, angle);
            sleep(dur);
            syarm.drive_comp_rel(i, -angle);
            println!("Done!");
            sleep(dur);
        }
    }
//

// Test meas
    #[test]
    fn test_meas() {
        let mut syarm = SyArm::load_json("res/syarm_const.json");
        syarm.init_meas();
        syarm.update_sim();
<<<<<<< HEAD

        // let raw_path = syarm.gen_lin_path(START, END, DECO, ACC).unwrap();
        // let drive_path = syarm.calc_drive_paths(&raw_path, VEL, (END - START).length());

        // // print_stepper_path(&non_corrected);

        // let paths = syarm.run_path_correction(drive_path);
        // let path = &paths[0];
        
        // print_stepper_path(path);
=======
    
        // syarm.debug_pins();
    
        println!("Running measurement tests ... ");
    
        sleep(Duration::from_secs(1));
    
        syarm.measure(2).unwrap();
>>>>>>> cb8048fca6b27c64dfd902df7c160d85b4b1584f
    }
// 

// Test linear move
    // fn print_stepper_path(path : &StepperPath) {
    //     println!("| index\t| dt\t| phi\t| omega\t| alpha\t|");
    //     for i in 0 .. path.dts.len() {
    //         println!("|{}\t|{}\t|{}\t|{}\t|{}\t|", i, path.dts[i], path.phis[i], path.omegas[i], path.alphas[i]);
    //     }
    // }

    // #[test]
    // fn test_linear_move() {
    //     let mut syarm = SyArm::load_json("res/syarm_const.json");  

    //     // Constants
    //     const ACC : f32 = 10.0;
    //     const DECO : f32 = 0.0;
    //     const VEL : f32 = 100.0;
    //     const START : Vec3 = Vec3::new(0.0, 380.0, 400.0);
    //     const END : Vec3 = Vec3::new(-200.0, 200.0, 300.0);

    //     let angles : Phis = syarm.get_with_fixed_dec(START, DECO);
    //     // let vecs = syarm._by_phis(&angles);

    //     syarm.write_position(&syarm.gammas_for_phis(angles));
    //     syarm.update_sim();

    //     // let raw_path = syarm.gen_lin_path(START, END, DECO, ACC).unwrap();
    //     // let drive_path = syarm.calc_drive_paths(&raw_path, VEL, (END - START).length());

    //     // print_stepper_path(&non_corrected);

    //     // let paths = syarm.run_path_correction(drive_path);
    //     // let path = &paths[0];
        
    //     // print_stepper_path(path);
    // }
//