use mlua::Lua;

use crate::tests::TestXYRobot;

#[test]
fn rockot() {
    let lua = Lua::new();

    let rob = TestXYRobot::new_simple();

    lua.globals().set("rob", rob).unwrap();

    lua.load(
        include_str!("lua/test.lua")
    ).exec().unwrap();
}