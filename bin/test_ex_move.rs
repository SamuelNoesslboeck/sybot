use syarm_lib::SyArm;

fn main() {
    let mut syarm = SyArm::load_json("res/syarm_const.json");
    syarm.init_meas();

    println!("Measuring ... ");

    syarm.update_sim();
    syarm.measure(2); 
    syarm.update_sim();

    println!("Exact positioning");
    syarm.drive_a3_abs(0.0);
    println!("Done!");
}