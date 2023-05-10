use sybot_pkg::Package;

fn main() {
    let args : Vec<String> = std::env::args().collect();
    
    if args.len() < 2 {
        eprintln!("Bad syntax!");
        eprintln!("dump [path] ... ");
        eprintln!(" -> [path]: Package path to dump");
        return;
    }

    let path = &args[1];
    
    match Package::load(path) {
        Ok(pkg) => { dbg!(pkg); },
        Err(err) => {
            eprintln!("{}", err);
        }
    };
}