use alloc::rc::Rc;
use core::cell::RefCell;
use std::sync::Mutex;

use actix_web::dev::{ServiceFactory, ServiceRequest};
use actix_web::{web, App, HttpResponse, HttpRequest, Responder};
use actix_web_actors::ws;

use glam::Vec3;
use stepper_lib::units::*;

use crate::{ActRobot,Interpreter};
use crate::remote::PushRemote;
use crate::robot::SafeRobot;

// Submodules
pub mod conf;
pub mod websocket;
// 

pub struct AppData<R : ActRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> {
    pub rob : Rc<RefCell<R>>,
    pub intpr : Rc<RefCell<dyn Interpreter<R, Result<serde_json::Value, R::Error>>>>,

    pub phis : [Phi; COMP],
    pub pos : Vec3
}

impl<R : ActRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> PushRemote<COMP> 
    for AppData<R, COMP, DECO, DIM, ROT>
{
    fn pub_phis(&mut self, phis : &[Phi; COMP]) -> Result<(), crate::Error> {
        self.phis = *phis;

        Ok(())
    }
}

// Paths
    mod api 
    {
        use super::*;

        pub async fn conf<R : ActRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (data_mutex : web::Data<Mutex<AppData<R, COMP, DECO, DIM, ROT>>>) -> impl Responder 
        {
            let data = data_mutex.lock().unwrap();
            let rob = data.rob.borrow();
            HttpResponse::Ok().content_type("application/json").body(
                serde_json::to_string_pretty(&rob.json_conf()).unwrap()
            )
        }   

        pub async fn pos<R : ActRobot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
            (_ : web::Data<Mutex<AppData<R, COMP, DECO, DIM, ROT>>>) -> impl Responder {
            // let data = data_mutex.lock().unwrap();
            let point = glam::Vec3::ZERO;
            HttpResponse::Ok().content_type("application/json").body(serde_json::to_string_pretty(&point.to_array()).unwrap())
        }   
    }

    async fn intpr<R : ActRobot<COMP, DECO, DIM, ROT, Error = std::io::Error> + 'static, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
        (data_mutex : web::Data<Mutex<AppData<R, COMP, DECO, DIM, ROT>>>, req: HttpRequest, stream: web::Payload) -> impl Responder {
        ws::start(websocket::WSHandler {
            data: data_mutex.into_inner().clone()
        }, &req, stream)
    }
//

pub fn create_robot_webserver<R : SafeRobot<COMP, DECO, DIM, ROT, Error = std::io::Error> + 'static, T, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize>
    (rob : Rc<RefCell<R>>, intp : Rc<RefCell<dyn Interpreter<R, Result<serde_json::Value, R::Error>>>>, app : App<T>) -> App<T>
    where
        T : ServiceFactory<ServiceRequest, Config = (), Error = actix_web::Error, InitError = ()> 
{
    let data = web::Data::new(Mutex::new(AppData {
        rob: rob.clone(), 
        intpr: intp.clone(),

        phis: [Phi::ZERO; COMP],
        pos: Vec3::ZERO
    }));

    rob.borrow_mut().add_remote(Box::new(data.clone().into_inner()));

    app.app_data(data)
    .service(web::scope("/")
        .route("/intpr", web::get().to(intpr::<R, COMP, DECO, DIM, ROT>))
    )
    .service(web::scope("/api")
        .route("/conf", web::get().to(api::conf::<R, COMP, DECO, DIM, ROT>))
        .route("/pos", web::get().to(api::pos::<R, COMP, DECO, DIM, ROT>))
    )
}