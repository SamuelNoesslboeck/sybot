# HTTP

<details>
<summary>
Click to show Cargo.toml
</summary>

```toml
# ...

[dependencies]
# Include the library configured for the raspberry pi and with http enabled
sybot = { version = "0.8.2", features = [ "rasp", "http" ] }

# ...
```
</details>
<p></p>

```rust
extern crate alloc;

use core::cell::RefCell;

use alloc::rc::Rc;
use actix_web::{HttpServer, App};

use sybot::{JsonConfig, SyArm, Robot};
use sybot::intpr::gcode::init_intpr;
use sybot::http::create_robot_webserver;

#[actix::main]
async fn main() -> Result<(), std::io::Error> {    
    HttpServer::new(move || {
        // Load the standard-partlibs in order to use motor names as data
        //
        // ```json
        // "ctrl": {
        //     "consts": "MOT_17HE15_1504S",    // Motor name, see
        // // <https://docs.rs/syact/0.11.1/syact/data/struct.StepperConst.html#associatedconstant.MOT_17HE15_1504S>
        //     "pin_dir": 17,
        //     "pin_step": 26
        // },
        // ```
        let libs = sybot::partlib::create_std_libs();

        // Create the robot out of the [configuration file]
        // (https://github.com/SamuelNoesslboeck/sybot/blob/master/res/SyArm_Mk1.conf.json)
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
```