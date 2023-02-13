use stepper_lib::JsonConfig;
use sybot_lib::{init_interpreter, SyArm, SyArmError, ErrType, Robot};

// use std::{time::Duration, thread::sleep, f32::consts::PI};

fn main() -> std::io::Result<()> {
    let mut syarm = SyArm::from_conf(
        JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
    )?;

    // DEBUG
        // Select pencil
        syarm.set_tool_id(1);
    // 

    let mut intpr = init_interpreter(syarm);

    let args : Vec<String> = std::env::args().collect();

    println!("SyArm - GCode Interpreter"); 
    
    if args.len() > 1 {
        println!(" -> Interpreting file '{}'", args[1]);
        intpr.interpret_file(args[1].as_str(), 
            |_| { Err(SyArmError::new_simple(ErrType::GCodeFuncNotFound)) });
    }
    
    loop {
        let mut line = String::new();
        std::io::stdin().read_line(&mut line).unwrap();

        match intpr.interpret(line.as_str(), |_| { Err(SyArmError::new_simple(ErrType::GCodeFuncNotFound)) }).first().unwrap() {
            Ok(j) => { 
                println!("{}", j.to_string());
            },
            Err(err) => {
                println!("{}", err);
            }
        }
    }
}