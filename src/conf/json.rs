use alloc::sync::Arc;

use glam::Vec3;
use serde::{Serialize, Deserialize};

use stepper_lib::{Tool, SyncComp, SyncCompGroup};
use stepper_lib::data::LinkedData;

use crate::MachineConfig;
use crate::conf::ConfigElement;
use crate::conf::partlib;


#[derive(Serialize, Deserialize)]
pub struct JsonConfig
{
    pub name : String,
    pub conf_version : String, 
    pub author : Option<String>, 

    pub lk : LinkedData,

    pub anchor : Option<[f32; 3]>,
    pub dims : Option<Vec<[f32; 3]>>,
    pub axes : Option<Vec<[f32; 3]>>,

    pub comps : Vec<ConfigElement>,
    pub tools : Vec<ConfigElement>
}

impl JsonConfig 
{
    pub fn get_comps<const N : usize>(&self) -> Result<[Box<dyn SyncComp>; N], std::io::Error> {
        let mut comps = vec![];

        for i in 0 .. N {
            let mut comp : Box<dyn SyncComp> = self.comps[i].get_comp().unwrap();


            match self.comps[i].meas {
                Some(meas) => {
                    comp.init_meas(meas.pin);
                },
                None => { }
            }; 

            comps.push(comp);
        }

        let mut comp_group : [Box<dyn SyncComp>; N] = comps.try_into().unwrap();
        comp_group.write_link(self.lk.clone());

        Ok(comp_group)
    }

    pub fn get_async_comps<const N : usize>(&self) -> Result<[Box<dyn SyncComp>; N], std::io::Error> {
        let mut comps = vec![];

        for i in 0 .. N {
            let mut comp : Box<dyn SyncComp> = self.comps[i].get_comp().unwrap();


            match self.comps[i].meas {
                Some(meas) => {
                    comp.init_meas(meas.pin);
                },
                None => { }
            }; 

            comps.push(comp);
        }

        let mut comp_group : [Box<dyn SyncComp>; N] = comps.try_into().unwrap();
        comp_group.write_link(self.lk.clone());

        Ok(comp_group)
    }

    pub fn get_machine<const N : usize, const D : usize, const A : usize>(&self) -> Result<MachineConfig<N, D, A>, std::io::Error> {
        if self.comps.len() != N {
            return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, 
                format!("Not enough components for machine! [Required: {}, Given: {}]", N, self.comps.len())))
        }

        let mut mach : MachineConfig<N, D, A> = Default::default();
        
        // Init
        mach.name = self.name.clone();
        mach.lk = Arc::new(self.lk.clone());

        mach.anchor = match self.anchor {
            Some(anchor_raw) => Vec3::from(anchor_raw),
            None => Default::default()
        };

        mach.dims = match &self.dims {
            Some(dims) => match dims.iter().map(|axis_raw| Vec3::from(*axis_raw)).collect::<Vec<Vec3>>().try_into() {
                Ok(val) => val, 
                Err(_) => return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData, format!("Not enough dimensions defined for machine! [Required: {}, Given: {}]", D, dims.len())))
            },
            None => return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData, format!("Not enough dimensions defined for machine! [Required: {}, Given: 0]", D)))
        };

        mach.axes = match &self.axes {
            Some(axes) => match axes.iter().map(|axis_raw| Vec3::from(*axis_raw)).collect::<Vec<Vec3>>().try_into() {
                Ok(val) => val, 
                Err(_) => return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData, format!("Not enough axes defined for machine! [Required: {}, Given: {}]", A, axes.len())))
            },
            None => return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData, format!("Not enough axes defined for machine! [Required: {}, Given: 0]", A)))
        };

        mach.tools = self.tools.iter().map(
            |tool_raw| tool_raw.get_tool().unwrap()
        ).collect();
        
        for i in 0 .. N {
            mach.ang[i] = self.comps[i].ang;
            mach.sim[i] = self.comps[i].sim;

            // Collected arrays
            match self.comps[i].limit {
                Some(lim) => { 
                    

                    mach.vels[i] = lim.vel;
                    mach.limit[i] = lim;
                }, 
                None => { }
            };

            match self.comps[i].meas {
                Some(meas) => {
                    mach.meas_dist[i] = meas.dist;
                    mach.home[i] = meas.set_val;
                    mach.meas[i] = meas;
                },
                None => { }
            }; 
        }

        Ok(mach)
    }

    pub fn to_string_pretty(&self) -> String {
        serde_json::to_string_pretty(self).unwrap()
    }

    pub fn comp_id_by_name(&self, name : &str) -> Option<usize> {
        for i in 0 .. self.comps.len() {
            if self.comps[i].name == name {
                return Some(i)
            }
        }

        None
    }

    pub fn tool_id_by_name(&self, name : &str) -> Option<usize> {
        for i in 0 .. self.tools.len() {
            if self.tools[i].name == name {
                return Some(i)
            }
        }

        None
    }

    // File I/O
        pub fn save_to_file(&self, path : &str) {
            std::fs::write(path, self.to_string_pretty()).unwrap()
        }

        pub fn read_from_file(libs : &partlib::StdLibs, path : &str) -> Self {
            let mut obj : serde_json::Value = serde_json::from_str(std::fs::read_to_string(path).unwrap().as_str()).unwrap();
            partlib::scan_obj(libs, &mut obj, &String::from("consts"));     // Replace std-strings

            serde_json::from_value(obj).unwrap()
        }
    // 

    // Debug
        
    // 
}

pub fn create_conf_comps<const N : usize>(comps : &[Box<dyn SyncComp>; N]) -> Vec<ConfigElement> {
    let mut values = vec![];
    for i in 0 .. N {
        values.push(
            ConfigElement::from(&comps[i])
        );
    }
    values
}

pub fn create_conf_tools(tools : &Vec<Box<dyn Tool + Send>>) -> Vec<ConfigElement> {
    let mut values = vec![];
    for tool in tools {
        values.push(
            ConfigElement::from(
                tool
            )
        );
    }
    values
}