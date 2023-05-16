use sybot_pkg::Package;
use sybot_robs::{TheoRobot, BasicStepperRobot};

fn main() -> Result<(), sybot_robs::Error> {
    let pkg = Package::load("../assets/SyArm_Mk1")?;
    let theo = TheoRobot::try_from(pkg)?;

    println!("Parsing 'Theorobot' ... ");
    dbg!(theo);

    let pkg = Package::load("../assets/SyArm_Mk1")?;
    let simple : SimpleRobot<4> = <SimpleRobot<4> as BasicRobot<4>>::from_package(pkg)?;

    println!("Parsing 'SimpleRobot' ... ");
    dbg!(simple);

    Ok(())
}