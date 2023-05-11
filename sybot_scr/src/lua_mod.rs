use core::cell::RefCell;

use alloc::rc::Rc;
use glam::Vec3;
use mlua::{Lua, Result, Table, UserData, LuaSerdeExt};
use serde::{Serialize, Deserialize};
use sybot_robs::InfoRobot;

#[derive(Clone)]
struct RobStorage {
    pub rob : Rc<RefCell<dyn InfoRobot>>
}

impl UserData for RobStorage { }

#[derive(Clone, Serialize, Deserialize)]
struct LVec3 {
    pub x : f32,
    pub y : f32,
    pub z : f32
}

impl Into<Vec3> for LVec3 {
    fn into(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }
}

impl UserData for LVec3 { }

pub fn init_lib(lua : &Lua) -> Result<Table> {
    let exports = lua.create_table()?;
    let globals = lua.globals();

    globals.set("load_rob", lua.create_function(|lua, path : String| {
        let libs = create_std_libs();
        let globals = lua.globals();
    
        globals.set("__rob", RobStorage {
            rob: Rc::new(RefCell::new(SyArm::from_conf(JsonConfig::read_from_file(&libs, &path)).unwrap()))
        })?;     

        Ok(())
    })?)?;

    globals.set("print_rob", lua.create_function(|lua, ()| {
        let globals = lua.globals();
        let robs : RobStorage = globals.get("__rob")?;
        
        robs.rob.borrow().brob().print_conf_header();

        Ok(())
    })?)?;

    // Movement
        // globals.set("measure", lua.create_function(|lua, ()| {
        //     let globals = lua.globals();
        //     let robs : RobStorage = globals.get("__rob")?;
            
        //     // robs.rob.borrow().brob().print_conf_header();

        //     Ok(())
        // })?)?;

        // globals.set("move_j", lua.create_function(|lua, ()| {
        //     // let globals = lua.globals();
        //     // let robs : RobStorage = globals.get("__rob")?;
            
        //     // robs.rob.borrow().brob().print_conf_header();

        //     Ok(())
        // })?)?;
    //

    // Tools
        globals.set("set_tool", lua.create_function(|lua, index : usize| {
            let globals = lua.globals();
            let rob_storage : RobStorage = globals.get("__rob")?;
            let mut rob = rob_storage.rob.borrow_mut();
            
            if let Some(tool) = rob.set_tool_id(index) {
                lua.to_value(&tool.get_json())
            } else {
                Err(mlua::Error::RuntimeError("No tool found for the given index!".to_owned()))
            }
        })?)?;
    // 

    Ok(exports)
}

