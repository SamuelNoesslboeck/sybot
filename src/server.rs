use std::sync::{Arc, Mutex};
use std::time::Instant;

use actix::{Actor, StreamHandler};
use actix_web::{get, web, App, HttpResponse, HttpServer, Responder, HttpRequest};
use actix_web_actors::ws;

use serde_json::json;

use crate::{SyArm, SyArmResult, SyArmError, Robot, JsonConfig};
use crate::{Interpreter, init_interpreter};

struct AppData {
    pub intpr : Interpreter<SyArm, SyArmResult<serde_json::Value>>
}

struct MyWs {
    pub data : Arc<Mutex<AppData>>
}

impl Actor for MyWs {
    type Context = ws::WebsocketContext<Self>;
}

impl StreamHandler<Result<ws::Message, ws::ProtocolError>> for MyWs {
    fn handle(&mut self, msg: Result<ws::Message, ws::ProtocolError>, ctx: &mut Self::Context) {
        match msg {
            Ok(ws::Message::Text(text)) => { 
                let inst = Instant::now();

                let mut data = self.data.lock().unwrap();
                let results = data.intpr.interpret(&text, |gc_line| {
                    Err(SyArmError::new(
                        format!("GCode function [{}{}] not found!", gc_line.mnemonic(), gc_line.major_number()).as_str(), 
                            crate::types::ErrType::GCodeFuncNotFound))
                });

                let mut results_json : Vec<serde_json::Value> = vec![];

                for res in results {
                    match res {
                        Ok(val) => results_json.push(val),
                        Err(err) => results_json.push(json![{
                            "code": err.err_type as u64,
                            "msg": err.msg
                        }])
                    }
                }

                let el_time = inst.elapsed().as_secs_f32();

                println!(" -> GCode: {} Lines executed in {}s", results_json.len(), el_time);
                
                ctx.text(serde_json::to_string(&json!({
                    "el": el_time,
                    "len": results_json.len(),
                    "res": results_json
                })).unwrap())
            },
            _ => ()
        }
    }
}

// Paths
    #[get("/api/pos")]
    async fn api_pos(data_mutex : web::Data<Mutex<AppData>>) -> impl Responder {
        let data = data_mutex.lock().unwrap();
        let point = data.intpr.mach.vars.point;
        HttpResponse::Ok().content_type("application/json").body(serde_json::to_string_pretty(&point.to_array()).unwrap())
    }

    #[get("/conf")]
    async fn cons(data_mutex : web::Data<Mutex<AppData>>) -> impl Responder {
        let data = data_mutex.lock().unwrap();
        HttpResponse::Ok().content_type("application/json").body(serde_json::to_string_pretty(&data.intpr.mach.conf).unwrap())
    }

    #[get("/intpr")]
    async fn intpr(data_mutex : web::Data<Mutex<AppData>>, req: HttpRequest, stream: web::Payload) -> impl Responder {
        ws::start(MyWs {
            data: data_mutex.into_inner().clone()
        }, &req, stream)
    }
//

const ADDR : (&str, u16) = ("127.0.0.1", 8080);

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    println!("[SyArm - Webserver]");
    println!("(c) Samuel Nösslböck / SY 2022");
    println!("\n -> Hosting on ({}, {})", ADDR.0, ADDR.1);

    HttpServer::new(|| {
        App::new()
            .app_data(web::Data::new(Mutex::new(AppData {
                intpr: init_interpreter(SyArm::from_conf(JsonConfig::read_from_file("res/syarm_const.json")).unwrap())
            })))
            .service(api_pos)
            .service(cons)
            .service(intpr)
    })
    .bind(ADDR)?
    .run()
    .await
}