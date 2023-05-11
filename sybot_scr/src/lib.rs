extern crate alloc;

use mlua::{Lua, Result, Table};

// Submods
    mod lua_mod;
// 

#[mlua::lua_module]
fn sybot_lib(lua : &Lua) -> Result<Table> {
    lua_mod::init_lib(lua)
}