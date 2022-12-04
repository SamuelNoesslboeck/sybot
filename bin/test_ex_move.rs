use syarm_lib::SyArm;

use std::{time::Duration, thread::sleep, f32::consts::PI};

fn main() {
    let mut syarm = SyArm::load_json("res/syarm_const.json");
    syarm.init_meas();

    println!("Measuring ... ");

    syarm.update_sim();
    syarm.measure(2); 
    syarm.update_sim();

    // sleep(Duration::from_secs_f64(3.0));

    println!("Position A2: {}", syarm.ctrl_a2.get_gamma());

    println!("Exact positioning");
    syarm.drive_a2_abs(PI / 2.0);
    println!("Done!");

    println!("Position A2: {}", syarm.ctrl_a2.get_gamma());

    // ARM III

    println!("Position A3: {}", syarm.ctrl_a3.get_pos());

    println!("Exact positioning");
    syarm.drive_a3_abs(0.0);
    println!("Done!");

    println!("Position A3: {}", syarm.ctrl_a3.get_pos());
}