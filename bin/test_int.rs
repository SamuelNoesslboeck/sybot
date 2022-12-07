use syarm_lib::{init_interpreter, SyArm};

// use std::{time::Duration, thread::sleep, f32::consts::PI};

fn main() {
    let mut syarm = SyArm::load_json("res/syarm_const.json");
    syarm.init_meas();

    let mut intpr = init_interpreter(syarm);

    loop {
        let mut line = String::new();
        print!("GCode:// ");
        std::io::stdin().read_line(&mut line).unwrap();

        intpr.interpret(line.as_str());
    }
}