extern crate alloc;

use core::cell::RefCell;

use alloc::rc::Rc;
use actix_web::{HttpServer, App};

use sybot_lib::{JsonConfig, SyArm, Robot};
use sybot_lib::intpr::gcode::init_intpr;
use sybot_lib::http::create_robot_webserver;

#[actix::main]
async fn main() -> Result<(), std::io::Error> {    
    HttpServer::new(move || {
        // Load the standard-partlibs in order to use motor names as data
        //
        // ```json
        // "ctrl": {
        //     "consts": "MOT_17HE15_1504S",    // Motor name, see
        // // <https://docs.rs/stepper_lib/0.11.1/stepper_lib/data/struct.StepperConst.html#associatedconstant.MOT_17HE15_1504S>
        //     "pin_dir": 17,
        //     "pin_step": 26
        // },
        // ```
        let libs = sybot_lib::partlib::create_std_libs();

        // Create the robot out of the [configuration file]
        // (https://github.com/SamuelNoesslboeck/sybot_lib/blob/master/res/SyArm_Mk1.conf.json)
        let mut syarm = SyArm::from_conf(
            JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json")
        )?;
        
        // Create a new interpreter in a [std::rc::Rc]
        let intpr = Rc::new(RefCell::new(init_intpr()));

        // Create the webserver
        create_robot_webserver::<SyArm, _, 4, 1, 4, 4>(syarm, intpr, App::new())
    }).bind(("127.0.0.1", 8080))?   // Bind the webserver 
    .run()
    .await
}