use syarm_lib::SyArm;

use std::{time::Duration, thread::sleep, f32::consts::PI}; 

fn main() {
    let mut syarm = SyArm::load("res/syarm_const.json");
    syarm.init_meas();

    syarm.debug_pins();

    println!("Running measurement tests ... ");

    syarm.measure(5);
}