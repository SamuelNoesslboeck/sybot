use syarm_lib::{init_interpreter, SyArm, SyArmError, ErrType};

// use std::{time::Duration, thread::sleep, f32::consts::PI};

fn main() {
    let mut syarm = SyArm::load_json("res/syarm_const.json");
    syarm.init_meas();

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