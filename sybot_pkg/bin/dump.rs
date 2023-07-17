use std::time::Instant;

use sybot_pkg::Package;

fn main() {
    let args : Vec<String> = std::env::args().collect();
    // let args : Vec<&str> = vec![ "dump.exe", "assets/SyArm_Mk1" ];
    
    if args.len() < 2 {
        eprintln!("Bad syntax!");
        eprintln!("dump [path] ... ");
        eprintln!(" -> [path]: Package path to dump");
        return;
    }

    let path = &args[1];

    let inst = Instant::now();
    let pkg_res = Package::load(path);  
    let el = inst.elapsed().as_secs_f32();
    
    match pkg_res {
        Ok(pkg) => { 
            dbg!(&pkg); 
            println!(" => Loaded pkg '{}' in {}s", &pkg.info.name, el);
        },
        Err(err) => {
            eprintln!("{}", err);
        }
    };
}