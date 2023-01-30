// TODO: Rewrite into tests

// use stepper_lib::Component;
// use syarm_lib::SyArm;

// use std::{f32::consts::PI};

// fn main() {
//     let mut syarm = SyArm::load_json("res/syarm_const.json");
//     syarm.init_meas();

//     println!("Measuring ... ");

//     syarm.update_sim();
//     syarm.measure(2).unwrap(); 
//     syarm.update_sim();

//     // sleep(Duration::from_secs_f64(3.0));

//     // println!("Position A1: {}", syarm.ctrl_a1.get_gamma());

//     println!("Exact positioning");
//     syarm.drive_a1_abs(syarm.gamma_a1(PI / 2.0));
//     println!("Done!");

//     println!("Position A1: {}", syarm.ctrl_a1.get_dist());


//     println!("Exact positioning");
//     syarm.drive_a2_abs(syarm.gamma_a2(-PI / 2.0));
//     println!("Done!");

//     println!("Position A2: {}", syarm.ctrl_a2.get_dist());

//     // ARM III

//     println!("Position A3: {}", syarm.ctrl_a3.get_dist());

//     println!("Exact positioning");
//     syarm.drive_a3_abs(0.0);
//     println!("Done!");

//     println!("Position A3: {}", syarm.ctrl_a3.get_dist());

//     // println!("Exact positioning");
//     // syarm.drive_base_abs(PI / 8.0);
//     // println!("Done!");

//     // println!("Position Base: {}", syarm.ctrl_a3.get_dist());

//     let phis = syarm.all_phis();
//     dbg!(syarm.points_by_phis(&phis));
// }