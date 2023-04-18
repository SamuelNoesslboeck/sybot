use std::time::Instant; 

use stepper_lib::units::*;

use sybot_lib::{mqtt::Publisher, PushRemote, Robot, ActRobot};

#[test]
fn mqtt() -> Result<(), std::io::Error> {
    let mut pb = Publisher::new("mqtt://syhub:1883")?;
    pb.connect()?;
    
    loop {
        let inst = Instant::now();
        pb.pub_phis(&[Phi(0.1); 4])?; 
        println!("Sent message! {}", inst.elapsed().as_secs_f32());

        std::thread::sleep(core::time::Duration::from_secs(1));
    }
}

#[test]
fn remotes() -> Result<(), std::io::Error> {
    let libs = crate::partlib::create_std_libs();
    let mut syarm = crate::SyArm::from_conf(
        crate::JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
    )?;

    let pb = Publisher::new("mqtt://syhub:1883")?;
    pb.connect()?;
    
    syarm.add_remote(Box::new(pb));

    syarm.update(Some(&[Phi(0.5); 4]))?;

    Ok(())
}