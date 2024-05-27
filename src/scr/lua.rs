use mlua::UserData;
use syact::{SyncActuator, SyncActuatorGroup};
use syact::act::stepper::StepperActuatorGroup;
use syact::act::StepperActuator;
use syact::math::movements::DefinedActuator;

use crate::{Robot, StepperRobot};

pub fn load_rob_fields<'lua, G, R, T, F, const C : usize>(fields : &mut F)
where
    G : SyncActuatorGroup<T, C>,
    R : Robot<G, T, C> + 'static,
    T : SyncActuator + DefinedActuator + ?Sized + 'static,
    F : mlua::UserDataFields<'lua, R>
{
    fields.add_field_method_get("gammas", |_, this| Ok(this.gammas().map(|g| g.0)));
    fields.add_field_method_get("phis", |_, this| Ok(this.phis().map(|g| g.0)));

}

pub fn load_rob_methods<'lua, G, R, T, M, const C : usize>(_methods : &mut M)
where
    G : SyncActuatorGroup<T, C>,
    R : Robot<G, T, C> + 'static,
    T : SyncActuator + DefinedActuator + ?Sized + 'static,
    M : mlua::UserDataMethods<'lua, R>
{
    // methods.add_method(name, method)
}

impl<G, T, const C : usize> UserData for StepperRobot<G, T, C>
where
    G : StepperActuatorGroup<T, C> + 'static,
    T : StepperActuator + ?Sized + 'static 
{
    fn add_fields<'lua, F: mlua::UserDataFields<'lua, Self>>(fields: &mut F) {
        load_rob_fields(fields)
    }

    fn add_methods<'lua, M: mlua::UserDataMethods<'lua, Self>>(methods: &mut M) {
        load_rob_methods(methods)
    }
}