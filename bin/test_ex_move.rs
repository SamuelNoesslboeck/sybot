use syarm_lib::SyArm;

use std::{time::Duration, thread::sleep, f32::consts::PI}; 

fn main() {
    let mut syarm = SyArm::load_json("res/syarm_const.json");

    println!("Measuring ... ");

    syarm.update_sim();
    syarm.measure(2); 
    syarm.update_sim();

    println!("Exact positioning");
    syarm.drive_a3_abs(0.0);
    println!("Done!");
}