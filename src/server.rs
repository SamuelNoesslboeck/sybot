use std::sync::Mutex;

use actix_web::dev::{ServiceFactory, ServiceRequest};
use actix_web::{web, App, HttpResponse, HttpRequest, Responder};
use actix_web_actors::ws;

use crate::{JsonConfig, Robot, SafeRobot};
use crate::{Interpreter, init_intpr};

// Submodules
pub mod conf;
pub mod websocket;
// 

pub struct AppData<R : Robot<N, D>, const N : usize, const D : usize> {
    pub intpr : Interpreter<R, Result<serde_json::Value, R::Error>>
}

// Paths
    mod api 
    {
        use super::*;

        pub async fn conf<R : Robot<N, D>, const N : usize, const D : usize>(data_mutex : web::Data<Mutex<AppData<R, N, D>>>) -> impl Responder {
            let data = data_mutex.lock().unwrap();
            HttpResponse::Ok().content_type("application/json").body(serde_json::to_string_pretty(data.intpr.mach.json_conf()).unwrap())
        }   

        pub async fn pos<R : Robot<N, D>, const N : usize, const D : usize>(_ : web::Data<Mutex<AppData<R, N, D>>>) -> impl Responder {
            // let data = data_mutex.lock().unwrap();
            let point = glam::Vec3::ZERO;
            HttpResponse::Ok().content_type("application/json").body(serde_json::to_string_pretty(&point.to_array()).unwrap())
        }   
    }

    async fn intpr<R : Robot<N, D, Error = std::io::Error> + 'static, const N : usize, const D : usize>
        (data_mutex : web::Data<Mutex<AppData<R, N, D>>>, req: HttpRequest, stream: web::Payload) -> impl Responder {
        ws::start(websocket::WSHandler {
            data: data_mutex.into_inner().clone()
        }, &req, stream)
    }
//

pub fn create_robot_webserver<R : SafeRobot<N, D, Error = std::io::Error> + 'static, T, const N : usize, const D : usize>(conf : JsonConfig, app : App<T>) -> App<T>
    where
        T : ServiceFactory<ServiceRequest, Config = (), Error = actix_web::Error, InitError = ()> 
    {
    app.app_data(web::Data::new(Mutex::new(AppData {
        intpr: init_intpr(R::from_conf(conf).unwrap())
    })))
    .service(web::scope("/")
        .route("/intpr", web::get().to(intpr::<R, N>))
    )
    .service(web::scope("/api")
        .route("/conf", web::get().to(api::conf::<R, N>))
        .route("/pos", web::get().to(api::pos::<R, N>))
    )
}