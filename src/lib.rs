#![doc = include_str!("../README.md")]
#![crate_name = "sybot"]
// #![deny(missing_docs)]

/* Submodules */

use glam::{Vec2, Vec3};
/**/

use syact::{DefinedActuator, SyncActuator};
use syunit::prelude::Millimeters;
use syunit::{Factor, MetricMM, UnitSet, Unit};

pub fn ptp_factors<U : UnitSet, const C : usize>
    (act : [&dyn DefinedActuator<U>; C], pos_0 : [U::Position; C], pos : [U::Position; C], f_gen : Factor) -> [Factor; C] 
{
    let mut f = [Factor::MIN; C];
    let mut t = [U::Time::default(); C];
    let mut t_max = U::Time::default();     // will be 0, so times will always be greater

    for i in 0 .. C {
        t[i] = act[i].ptp_time_for_distance(pos_0[i], pos[i]);

        t_max = t[i].max(t_max);
    }

    for i in 0 .. C {
        f[i] = Factor::new(t[i] / t_max) * f_gen;
    }

    f
}

pub trait Descriptor {
    type SysCoord;
    type RobCoord;

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Result<Self::SysCoord, ()>;

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, ()>;
}

pub struct LinDesc2 { }

impl Descriptor for LinDesc2 {
    type SysCoord = Vec2;
    type RobCoord = (Millimeters, Millimeters);

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Result<Self::SysCoord, ()> {
        Ok(Vec2::new(rob.0.into(), rob.1.into()))
    }

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, ()> {
        Ok((Millimeters(sys.x), Millimeters(sys.y)))
    }
}

pub struct LinDesc3 { }

impl Descriptor for LinDesc3 {
    type SysCoord = Vec3;
    type RobCoord = (Millimeters, Millimeters, Millimeters);

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Result<Self::SysCoord, ()> {
        Ok(Vec3::new(rob.0.into(), rob.1.into(), rob.2.into()))
    }

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, ()> {
        Ok((Millimeters(sys.x), Millimeters(sys.y), Millimeters(sys.z)))
    }
}

pub trait Actuators {
    type RobCoord; 

    fn move_
}

pub trait PtpActuators : Actuators {
    fn move_ptp(&mut self, )
}

pub struct Actuators2<UX : UnitSet, UY : UnitSet, X : SyncActuator<UX>, Y : SyncActuator<UY>> {
    x : X,
    y : Y
}

pub struct Lin3Actuators<X : SyncActuator<MetricMM>, Y : SyncActuator<MetricMM>, Z : SyncActuator<MetricMM>> {
    x : X,
    y : Y,
    z : Z
}

pub struct Robot<A, D, S> 
where
    A : Actuators,
    D : Descriptor<RobCoord = A::RobCoord>
{
    pub acts : A,
    pub desc : D,
    pub stat : S
}

// type Lin2Robot<X, Y, S> = Robot<Lin2Actuators<X, Y>, LinDesc2, S>; 