use syarm_lib::SyArm;

use std::{time::Duration, thread::sleep}; 

fn main() {
    let mut syarm = SyArm::load_json("res/syarm_const.json");
    syarm.init_meas();
    syarm.update_sim();

    syarm.debug_pins();

    println!("Running measurement tests ... ");

    sleep(Duration::from_secs(1));

    syarm.measure_async(2);
}