use core::cell::RefCell;

use alloc::rc::Rc;
use mlua::{Lua, Result, Table, UserData};

use crate::{robot::SyArm, ActRobot, JsonConfig};
use crate::partlib::create_std_libs;

#[derive(Clone)]
struct RobStorage {
    pub rob : Rc<RefCell<SyArm>>
}

impl UserData for RobStorage { }

// fn lua_rob_func()

pub fn init_lib(lua : &Lua) -> Result<Table> {
    
    let exports = lua.create_table()?;
    let globals = lua.globals();

    globals.set("load_rob", lua.create_function(|lua, path : String| {
        let libs = create_std_libs();
        let globals = lua.globals();
    
        globals.set("__rob", RobStorage {
            rob: Rc::new(SyArm::from_conf(JsonConfig::read_from_file(&libs, &path)).unwrap())
        })?;     

        Ok(())
    })?)?;

    globals.set("print_rob", lua.create_function(|lua, ()| {
        let globals = lua.globals();
        let rob_storage : RobStorage = globals.get("__rob")?;

        rob_storage.rob.brob().print_conf_header();

        Ok(())
    })?)?;

    globals.set("print_rob", lua.create_function(|lua, ()| {
        let globals = lua.globals();
        let rob_storage : RobStorage = globals.get("__rob")?;
        
        rob_storage.rob.measure();

        Ok(())
    })?)?;

    Ok(exports)
}

