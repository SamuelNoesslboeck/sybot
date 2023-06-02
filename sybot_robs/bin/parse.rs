use sybot_pkg::Package;
use sybot_robs::{TheoRobot, StepperRobot};

pub type TestRobot<T> = StepperRobot::<T, 4>;

fn main() -> Result<(), sybot_robs::Error> {
    let pkg = Package::load("../assets/SyArm_Mk1")?;
    let theo = TheoRobot::<4>::try_from(pkg)?;

    println!("Parsing 'Theorobot' ... ");
    dbg!(theo);

    let pkg = Package::load("../assets/SyArm_Mk1")?;
    let _ = TestRobot::try_from(&pkg)?;

    // println!("{}", )

    Ok(())
}