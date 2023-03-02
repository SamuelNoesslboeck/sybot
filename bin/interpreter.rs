use colored_json::to_colored_json_auto;

use stepper_lib::JsonConfig;
use sybot_lib::{init_intpr, SyArm, ConfRobot};

fn main() -> std::io::Result<()> {
    let mut syarm = SyArm::from_conf(
        JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
    )?;

    // DEBUG
        // Select pencil
        syarm.set_tool_id(2);
    // 

    let mut intpr = init_intpr(syarm);

    let args : Vec<String> = std::env::args().collect();

    println!("SyArm - GCode Interpreter"); 
    
    if args.len() > 1 {
        println!(" -> Interpreting file '{}'", args[1]);
        intpr.interpret_file(args[1].as_str(), 
            |_| { Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, "Invalid GCode input")) });
    }
    
    loop {
        let mut line = String::new();

        println!("");
        std::io::stdin().read_line(&mut line).unwrap();

        match intpr.interpret(line.as_str(), |_| { Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, "Invalid GCode input")) }).first().unwrap() {
            Ok(j) => { 
                println!("{}", to_colored_json_auto(&j).unwrap());
            },
            Err(err) => {
                println!("{}", err);
            }
        }
    }
}