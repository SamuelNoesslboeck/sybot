use sybot_pkg::Package;
use sybot_robs::{TheoRobot, StepperRobot};

fn main() -> Result<(), sybot_robs::Error> {
    let pkg = Package::load("../assets/SyArm_Mk1")?;
    let theo = TheoRobot::<4>::try_from(pkg)?;

    println!("Parsing 'Theorobot' ... ");
    dbg!(theo);

    let pkg = Package::load("../assets/SyArm_Mk1")?;
    let simple = StepperRobot::<4>::try_from(pkg)?;

    println!("Parsing 'SimpleRobot' ... ");
    dbg!(simple);

    Ok(())
}