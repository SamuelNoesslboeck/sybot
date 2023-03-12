use std::{f32::consts::PI, thread::sleep, time::Duration};
use crate::*;

mod calc;
mod data;
mod movements;
mod robots;

// Test Arm
// 

// Test meas
    #[test]
    fn test_meas() -> std::io::Result<()> {
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        )?;

        syarm.update(None);
    
        // syarm.debug_pins();
    
        println!("Running measurement tests ... ");
    
        sleep(Duration::from_secs(1));
    
        syarm.measure()?;

        Ok(())
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