use core::cell::RefCell;
use core::ops::DerefMut;

use alloc::rc::Rc;
use glam::Vec3;
use mlua::{Lua, Result, Table, UserData, LuaSerdeExt};
use serde::{Serialize, Deserialize};
use syact::units::*;
use crate::rcs::Position;
use crate::robs::{BasicRobot, RobotDesc};

#[derive(Clone)]
pub struct RobStorage<const C : usize> {
    pub rob : Rc<RefCell<dyn BasicRobot<C>>>,
    pub desc : Rc<RefCell<dyn RobotDesc<C>>>
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

    globals.set("print_info", lua.create_function(|lua, ()| {
        let globals = lua.globals();
        let robs : RobStorage<C> = globals.get("rob")?;
        let rob = robs.rob.borrow();
        let desc = robs.desc.borrow();

        dbg!(rob.info());
        dbg!(desc.aconf().phis());

        Ok(())
    })?)?;

    // Data
        globals.set("aconf", lua.create_function(|lua, raw_phis : Vec<f32>| {
            let globals = lua.globals();
            let robs : RobStorage<C> = globals.get("rob")?;
            let mut desc = robs.desc.borrow_mut();
            let phis = raw_phis.iter().map(|p| Phi(*p)).collect::<Vec<Phi>>();

            desc.aconf_mut().configure(phis).unwrap();

            Ok(())
        })?)?;
    // 

    // Movement
        // globals.set("measure", lua.create_function(|lua, ()| {
        //     let globals = lua.globals();
        //     let robs : RobStorage = globals.get("__rob")?;
            
        //     // robs.rob.borrow().brob().print_conf_header();

        //     Ok(())
        // })?)?;

        globals.set("move_p_sync", lua.create_function(|lua, args: ( [f32; 3], Option<f32> )| {
            let globals = lua.globals();
            let robs : RobStorage<C> = globals.get("rob")?;
            let mut rob = robs.rob.borrow_mut();
            let mut desc = robs.desc.borrow_mut();

            let pos = Position::new(Vec3::from_array(args.0));
            let speed_f = args.1.unwrap_or(1.0);

            dbg!(rob.move_p_sync(desc.deref_mut(), pos, speed_f).unwrap());

            Ok(())
        })?)?;
    //

    // Tools
        globals.set("set_tool", lua.create_function(|lua, index : Option<usize>| {
            let globals = lua.globals();
            let robs : RobStorage<C> = globals.get("rob")?;
            let mut rob = robs.rob.borrow_mut();
            
            if let Some(tool) = rob.set_tool_id(index) {
                lua.to_value(&tool.get_json())
            } else {
                lua.to_value(&mlua::Value::Nil) 
            }
        })?)?;
    // 

    Ok(exports)
}

