use std::sync::Mutex;

use actix_web::dev::{ServiceFactory, ServiceRequest};
use actix_web::{web, App, HttpResponse, HttpRequest, Responder};
use actix_web_actors::ws;

use stepper_lib::units::*;

use crate::{JsonConfig, Robot, SafeRobot, Remote};
use crate::init_intpr;
use crate::gcode::Interpreter;

// Submodules
pub mod conf;
pub mod websocket;
// 

pub struct AppData<R : Robot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> {
    pub intpr : Interpreter<R, Result<serde_json::Value, R::Error>>,
    pub pos : [Phi; COMP]
}

impl<R : Robot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> Remote<COMP> 
    for AppData<R, COMP, DECO, DIM, ROT>
{
    fn pub_phis(&mut self, phis : &[Phi; COMP]) -> Result<(), crate::Error> {
        self.pos = *phis;

        Ok(())
    }

    fn pub_drive(&mut self) {
        todo!()
    }
}

impl<R : Robot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> Remote<COMP> 
    for Mutex<AppData<R, COMP, DECO, DIM, ROT>>
{
    fn pub_phis(&mut self, phis : &[Phi; COMP]) -> Result<(), crate::Error> {
        let mut data = self.lock().unwrap(); 

        data.pos = *phis;

        Ok(())
    }

    fn pub_drive(&mut self) {
        todo!()
    }
}

// Paths
    mod api 
    {
        use super::*;

        pub async fn conf<R : Robot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (data_mutex : web::Data<Mutex<AppData<R, COMP, DECO, DIM, ROT>>>) -> impl Responder {
            let data = data_mutex.lock().unwrap();
            HttpResponse::Ok().content_type("application/json").body(serde_json::to_string_pretty(&data.intpr.mach.json_conf()).unwrap())
        }   

        pub async fn pos<R : Robot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (_ : web::Data<Mutex<AppData<R, COMP, DECO, DIM, ROT>>>) -> impl Responder {
            // let data = data_mutex.lock().unwrap();
            let point = glam::Vec3::ZERO;
            HttpResponse::Ok().content_type("application/json").body(serde_json::to_string_pretty(&point.to_array()).unwrap())
        }   
    }

    async fn intpr<R : Robot<COMP, DECO, DIM, ROT, Error = std::io::Error> + 'static, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
        (data_mutex : web::Data<Mutex<AppData<R, COMP, DECO, DIM, ROT>>>, req: HttpRequest, stream: web::Payload) -> impl Responder {
        ws::start(websocket::WSHandler {
            data: data_mutex.into_inner().clone()
        }, &req, stream)
    }
//

pub fn create_robot_webserver<R : SafeRobot<COMP, DECO, DIM, ROT, Error = std::io::Error> + 'static, T, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
    (conf : JsonConfig, app : App<T>) -> App<T>
    where
        T : ServiceFactory<ServiceRequest, Config = (), Error = actix_web::Error, InitError = ()> 
{
    app.app_data(web::Data::new(Mutex::new(AppData {
        intpr: init_intpr(R::from_conf(conf).unwrap()),
        pos: [Phi::ZERO; COMP]
    })))
    .service(web::scope("/")
        .route("/intpr", web::get().to(intpr::<R, COMP, DECO, DIM, ROT>))
    )
    .service(web::scope("/api")
        .route("/conf", web::get().to(api::conf::<R, COMP, DECO, DIM, ROT>))
        .route("/pos", web::get().to(api::pos::<R, COMP, DECO, DIM, ROT>))
    )
}