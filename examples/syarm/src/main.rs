use rustyline::Editor;

use sybot::prelude::*;

mod syarm;
pub use syarm::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // let broker_addr = "syhub:1883".to_owned(); // std::env::var("SYARM_BROKER_ADDR").expect("SYARM_BROKER_ADDR must be set");

    println!("[SyArm ROS system] \nBasic robot operating system for the SyArm robot. (c) Samuel Noesslboeck 2023\n");
    println!("Initialising ... ");

    let pkg = Package::load(".")?;
    println!("- Loaded package: '{}'", pkg.info.name);

    let ( info, mut rob, mut desc, mut stat ) = pkg.unpack::<SyArmRob, SyArmDesc, SyArmStation>().unwrap();

    dbg!(info);

    // Remotes and interpreters
    let gcode = sybot::gcode::GCodeIntpr::init();

    // let mqtt = Box::new(
    //    sybot::mqtt::Publisher::new(&broker_addr, "syarm-rob-client")?);

    // if mqtt.connect().is_ok() {
    //     println!("- Successfully connected to MQTT-broker ({})", broker_addr);
    //     rob.add_remote(mqtt);
    // } else {
    //     eprintln!("- Failed to connect to broker");
    // }

    // Setup
        rob.setup()?;
        println!("- Setup complete");

        print!("- Setting home position... ");
        // rob.auto_meas()?;
        println!("done!");
    //

    println!("\nGCode interpreter");

    let mut editor = Editor::<(), _>::new().expect("Failed to make rustyline editor");

    loop {
        match editor.readline("> ") {
            Ok(input) => {
                editor.add_history_entry(&input)?;
                for res in gcode.interpret(&mut rob, &mut desc, &mut stat, &input) {
                    println!("{}\n", serde_json::to_string_pretty(&res?).unwrap());
                }
            },
            Err(_) => return Ok(()),
        }
    }
}
