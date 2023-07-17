use serde::{Serialize, Deserialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ServerConf
{
    #[serde(default = "ServerConf::default_addr")]
    pub addr : String,
    #[serde(default = "ServerConf::default_port")]
    pub port : u16
}

impl ServerConf {
    fn default_addr() -> String {
        String::from("127.0.0.1")
    }
    
    fn default_port() -> u16 {
        25106
    }
}