use actix_web::{HttpServer, App};
use sybot_lib::{server::create_robot_webserver, JsonConfig, SyArm};

#[actix::main]
async fn main() -> Result<(), std::io::Error> {

    HttpServer::new(|| {
        let libs = sybot_lib::partlib::create_std_libs();
        create_robot_webserver::<SyArm, _, 4, 1, 4, 4>(JsonConfig::read_from_file(&libs, "res/SyArm_Mk1.conf.json"), App::new())
    }).bind(("127.0.0.1", 8080))? 
    .run()
    .await
}