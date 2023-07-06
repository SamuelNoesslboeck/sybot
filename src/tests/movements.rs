use std::f32::consts::PI;
use std::thread::sleep;

use syact::units::*;

use crate::{ActRobot, Robot};

#[test]
fn all_axes() -> std::io::Result<()> {
    let libs = crate::partlib::create_std_libs();
    let mut syarm = crate::SyArm::from_conf(
        crate::JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
    )?;

    let dur = std::time::Duration::from_secs_f32(0.5);
    let angle = Delta(PI / 8.0);

    // syarm.update_sim();

    // syarm.debug_pins();

    println!("Running movement tests ... ");

    syarm.measure()?;
    
    for i in 0 .. 4 {
        print!(" -> Moving Component {i} ... ");
        syarm.drive_comp_rel(i, angle)?;
        sleep(dur);
        syarm.drive_comp_rel(i, -angle)?;
        println!("Done!");
        sleep(dur);
    }

    Ok(())
}