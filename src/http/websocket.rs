use std::sync::{Arc, Mutex};
use std::time::Instant;

use actix::{Actor, StreamHandler};
use actix_web_actors::ws;

use serde_json::json;

use crate::Robot;

use super::AppData;

pub struct WSHandler<R : Robot<COMP, DECO, DIM, ROT>, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> {
    pub data : Arc<Mutex<AppData<R, COMP, DECO, DIM, ROT>>>
}

impl<R : Robot<COMP, DECO, DIM, ROT> + 'static, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> Actor for WSHandler<R, COMP, DECO, DIM, ROT> {
    type Context = ws::WebsocketContext<Self>;
}

impl<R : Robot<COMP, DECO, DIM, ROT, Error = std::io::Error> + 'static, const COMP : usize, const DECO : usize, const DIM : usize, const ROT : usize> 
    StreamHandler<Result<ws::Message, ws::ProtocolError>> for WSHandler<R, COMP, DECO, DIM, ROT> 
{
    fn handle(&mut self, msg: Result<ws::Message, ws::ProtocolError>, ctx: &mut Self::Context) {
        match msg {
            Ok(ws::Message::Text(text)) => { 
                let inst = Instant::now();

                let mut data = self.data.lock().unwrap();
                let results = data.intpr.interpret(&text, |_| {
                    Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, ""))
                });

                let mut results_json : Vec<serde_json::Value> = vec![];

                for res in results {
                    match res {
                        Ok(val) => results_json.push(val),
                        Err(err) => results_json.push(json![{
                            "code": err.kind() as u64,
                            "msg": err.to_string()
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