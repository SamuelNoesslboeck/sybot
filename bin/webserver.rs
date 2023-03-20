extern crate alloc;

use core::cell::RefCell;

use actix_web::{HttpServer, App};

use alloc::rc::Rc;
use sybot_lib::intpr::gcode::init_intpr;
use sybot_lib::{JsonConfig, SyArm, Robot};
use sybot_lib::http::create_robot_webserver;

#[actix::main]
async fn main() -> Result<(), std::io::Error> {    
    HttpServer::new(move || {
        let libs = sybot_lib::partlib::create_std_libs();
        let syarm = Rc::new(RefCell::new(SyArm::from_conf(
            JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
        ).unwrap()));
    
        let intpr = Rc::new(RefCell::new(init_intpr()));

        create_robot_webserver::<SyArm, _, 4, 1, 4, 4>(syarm, intpr, App::new())
    }).bind(("127.0.0.1", 8080))? 
    .run()
    .await
}