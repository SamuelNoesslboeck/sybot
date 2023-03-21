use colored::Colorize; 
use colored_json::to_colored_json_auto;

use sybot_lib::{SyArm, Robot, JsonConfig};
use sybot_lib::intpr::Interpreter;
use sybot_lib::intpr::gcode::init_intpr;

const AUTHOR : &str = "Samuel Nösslböck (Sy)";
const VERSION : &str = "0.1.0";

fn main() -> std::io::Result<()> {
    std::env::set_var("RUST_BACKTRACE", "1");
    
    let libs = sybot_lib::partlib::create_std_libs();
    let mut syarm = SyArm::from_conf(
        JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
    )?;

    syarm.setup();
    syarm.setup_async();

    // Print Header
    println!("{}", "\n[SyArm - GCode Interpreter]".bold()); 
    println!("Version: {}, (c) {}\n", VERSION.truecolor(0xEA, 0x8C, 0x43), AUTHOR.truecolor(0xEA, 0x8C, 0x43));

    if cfg!(feature = "dbg-funcs") {
        syarm.print_conf_header();
        println!("");   // Newline for style
    }

    // DEBUG
        // Select "NoTool"
        syarm.set_tool_id(2);
    // 

    let intpr = init_intpr();

    let args : Vec<String> = std::env::args().collect();
    let mut lines = Vec::new();
    let mut lines_init_len = 0;

    #[allow(unused_assignments)]
    let mut file = String::new();
    
    if args.len() > 1 {
        println!(" -> Interpreting file {}", format!("\"{}\"", args[1]).green());

        file = std::fs::read_to_string(args[1].as_str()).unwrap();
        lines = file.split('\n').collect();
        lines_init_len = lines.len();
    } else {
        println!("{}", "\nInterpreter Terminal".bold())
    }
    
    println!("");

    loop {
        let mut line = String::new();

        let lines_len = lines.len();
        if lines_len > 0 {
            line = String::from(lines.remove(0));
        } else {
            std::io::stdin().read_line(&mut line).unwrap();
        }

        match intpr.interpret(&mut syarm, line.as_str()).first() {
            Some(res) => {
                if lines_len > 0 {
                    println!("{} {}", format!("{} |", lines_init_len - lines_len).bold(), line);
                }

                match res {
                    Ok(j) => { 
                        if !j.is_null() {
                            println!("{}\n", to_colored_json_auto(&j).unwrap());
                        }
                    },
                    Err(err) => {
                        println!("{}\n", err);
                    }
                }
            },
            None => { }
        }
    }
}