use std::time::Instant;

use glam::Vec3;
use rcs::Position;
// use stepper_lib::units::*;
use sybot_lib::prelude::*;

fn main() -> Result<(), sybot_lib::Error> {
    let inst = Instant::now();

    let pkg = Package::load("assets/SyArm_Mk1")?;
    let mut syarm = SyArm::try_from(pkg)?;

    println!(" => Created in {}s", inst.elapsed().as_secs_f32());

    let pos = Position::new(Vec3::new(330.0, 0.0, 400.0));

    println!("{:?}", syarm.desc.convert_pos(&mut syarm.rob, pos));

    // println!("SyArm");
    // println!(" => Segments: {:?}", syarm.desc.segments.calculate_end());

    // let inst = Instant::now();

    // // syarm.desc.update(&mut syarm.rob, &[ Phi(0.5), Phi(1.5), Phi(-1.5), Phi(0.2) ])?;
    // syarm.desc.update(&mut syarm.rob, &[ Phi::ZERO, Phi(1.5707963), Phi(-1.5707963), Phi::ZERO ])?;

    // println!(" => Created in {}s", inst.elapsed().as_secs_f32());
    // println!(" => Segments: {:?}", syarm.desc.segments.calculate_end());

    Ok(())
}