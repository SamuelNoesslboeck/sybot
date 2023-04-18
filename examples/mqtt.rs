use stepper_lib::units::*;

use sybot_lib::ActRobot;
use sybot_lib::conf::JsonConfig;
use sybot_lib::mqtt::Publisher;
use sybot_lib::robot::SyArm;

fn main() -> Result<(), std::io::Error> {
    let libs = sybot_lib::partlib::create_std_libs();
    let mut syarm = SyArm::from_conf(
        JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
    )?;

    let pb = Publisher::new("mqtt://syhub:1883")?;
    pb.connect()?;
    
    syarm.add_remote(Box::new(pb));

    syarm.update(Some(&[Phi(0.5); 4]))?;

    Ok(())
}