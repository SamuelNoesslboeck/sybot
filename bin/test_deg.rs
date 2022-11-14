use syarm_lib::SyArm;

use std::{time::Duration, thread::sleep, f32::consts::PI}; 

fn main() {
    let mut syarm = SyArm::load("res/syarm_const.json");
    let dur = Duration::from_secs_f32(0.5);
    let angle = PI / 4.0;

    syarm.debug_pins();

    println!("Running movement tests ... ");

    print!(" -> Moving base ... ");
    syarm.drive_base_rel(angle);
    sleep(dur);
    syarm.drive_base_rel(-angle);
    println!("Done!");

    print!(" -> Moving A1 ... ");
    syarm.ctrl_a1.cylinder.extend(10.0, 5.0);
    sleep(dur);
    syarm.ctrl_a1.cylinder.extend(-10.0, 5.0);
    println!("Done!");

    print!(" -> Moving A2 ... ");
    syarm.ctrl_a2.cylinder.extend(10.0, 5.0);
    sleep(dur);
    syarm.ctrl_a2.cylinder.extend(-10.0, 5.0);
    println!("Done!");

    print!(" -> Moving A3 ... ");
    syarm.drive_a3_rel(angle);
    sleep(dur);
    syarm.drive_a3_rel(-angle);
    println!("Done!");
}