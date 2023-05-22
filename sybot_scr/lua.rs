use core::cell::RefCell;

use alloc::rc::Rc;
use glam::Vec3;
use mlua::{Lua, Result, Table, UserData, LuaSerdeExt};
use serde::{Serialize, Deserialize};
use sybot_robs::BasicRobot;

#[derive(Clone)]
pub struct RobStorage<const C : usize> {
    pub rob : Rc<RefCell<dyn BasicRobot<C>>>
}

impl<const C : usize> UserData for RobStorage<C> { 
    fn add_fields<'lua, F: mlua::UserDataFields<'lua, Self>>(_fields: &mut F) {
        _fields.add_field_method_get("phis", |lua, this| { 
            let rob = this.rob.borrow();
            let phis = rob.phis();
            let farray = phis.iter().map(|p| p.0 );
            lua.create_sequence_from(farray)
        });
    }
}

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

pub fn init_lib<const C : usize>(lua : &Lua) -> Result<Table> {
    let exports = lua.create_table()?;
    let globals = lua.globals();

    // globals.set("print_rob", lua.create_function(|lua, ()| {
    //     let globals = lua.globals();
    //     let _ : RobStorage<C> = globals.get("__rob")?;

    //     // robs.rob.borrow().brob().print_conf_header();

    //     Ok(())
    // })?)?;

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
        globals.set("set_tool", lua.create_function(|lua, index : Option<usize>| {
            let globals = lua.globals();
            let rob_storage : RobStorage<C> = globals.get("rob")?;
            let mut rob = rob_storage.rob.borrow_mut();
            
            if let Some(tool) = rob.set_tool_id(index) {
                lua.to_value(&tool.get_json())
            } else {
                lua.to_value(&mlua::Value::Nil) 
            }
        })?)?;
    // 

    Ok(exports)
}

