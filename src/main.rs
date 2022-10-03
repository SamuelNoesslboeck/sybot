use syarm_lib::SyArm;

fn main() 
{
    let a = SyArm::load("./res/syarm.json");

    println!("{}", a.angles.delta_a1);
}