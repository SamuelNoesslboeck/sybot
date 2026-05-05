#![doc = include_str!("../README.md")]
#![crate_name = "sybot"]
#![allow(async_fn_in_trait)]
// #![deny(missing_docs)]

use core::marker::PhantomData;

use glam::{Vec2, Vec3};

/* Submodules */
    mod acts;
/**/

use syact::{ActuatorError, DefinedActuator, SyncActuator};
use syunit::prelude::Millimeters;
use syunit::{Factor, UnitSet, Unit};


pub trait Station {
    type Error;
}

pub enum RobotError<A : Actuators, D : Descriptor, S : Station> {
    ActsError(A::Error),
    DescError(D::Error),
    StatError(S::Error)
}

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
    type Error;

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Self::SysCoord;

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, Self::Error>;
}

pub struct LinDesc2 { }

impl Descriptor for LinDesc2 {
    type SysCoord = Vec2;
    type RobCoord = (Millimeters, Millimeters);
    type Error = ();    // TODO: Add error

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Self::SysCoord {
        Vec2::new(rob.0.into(), rob.1.into())
    }

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, Self::Error> {
        Ok((Millimeters(sys.x), Millimeters(sys.y)))
    }
}

pub struct LinDesc3 { }

impl Descriptor for LinDesc3 {
    type SysCoord = Vec3;
    type RobCoord = (Millimeters, Millimeters, Millimeters);
    type Error = ();

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Self::SysCoord {
        Vec3::new(rob.0.into(), rob.1.into(), rob.2.into())
    }

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, Self::Error> {
        Ok((Millimeters(sys.x), Millimeters(sys.y), Millimeters(sys.z)))
    }
}

pub trait Actuators {
    type RobCoord;
    type Error;

    fn pos(&self) -> Self::RobCoord;
}

pub trait PtpActuators : Actuators {
    async fn move_ptp(&mut self, pos : Self::RobCoord, f_gen : Factor) -> Result<(), Self::Error>;
}

pub struct Actuators2<UA : UnitSet, UB : UnitSet, A : SyncActuator<UA>, B : SyncActuator<UB>> {
    pub a : A,
    pub b : B,

    __pa : PhantomData<UA>,
    __pb : PhantomData<UB>
}

pub enum Actuators2Error<UA : UnitSet, UB : UnitSet> {
    ComponentA(ActuatorError<UA>),
    ComponentB(ActuatorError<UB>)
}

impl<UA, UB, A, B> Actuators for Actuators2<UA, UB, A, B>
where 
    UA : UnitSet,
    UB : UnitSet,
    A : SyncActuator<UA>,
    B : SyncActuator<UB>
{
    type RobCoord = (UA::Position, UB::Position);
    type Error = Actuators2Error<UA, UB>;

    fn pos(&self) -> Self::RobCoord {
        (self.a.pos(), self.b.pos())
    }
}

impl<UA, UB, A, B> PtpActuators for Actuators2<UA, UB, A, B>
where 
    UA : UnitSet,
    UB : UnitSet,
    A : SyncActuator<UA> + DefinedActuator<UA>,
    B : SyncActuator<UB> + DefinedActuator<UB>,

    UA::Time : From<UB::Time>
{
    async fn move_ptp(&mut self, pos : Self::RobCoord, f_gen : Factor) -> Result<(), Self::Error> {
        // Calculate factors
        let time : [UA::Time; 2] = [
            self.a.ptp_time_for_distance(self.a.pos(), pos.0),
            UA::Time::from(self.b.ptp_time_for_distance(self.b.pos(), pos.1))
        ];
        
        // Calculate maximum time
        let max_time = time[0].max(time[0]);

        let (res_a, res_b) = futures::join!(
            self.a.drive_abs(pos.0, f_gen * Factor::new(time[0] / max_time)),
            self.b.drive_abs(pos.1, f_gen * Factor::new(time[1] / max_time))
        );

        // Map the different errors                     
        res_a.map_err(|err| Actuators2Error::ComponentA(err))?;
        res_b.map_err(|err| Actuators2Error::ComponentB(err))?;

        Ok(())
    }
}

pub struct Robot<A, D, S> 
where
    A : Actuators,
    D : Descriptor<RobCoord = A::RobCoord>,
    S : Station
{
    pub acts : A,
    pub desc : D,
    pub stat : S
}

impl<A, D, S> Robot<A, D, S> 
where
    A : Actuators,
    D : Descriptor<RobCoord = A::RobCoord>,
    S : Station
{
    pub fn pos(&self) -> D::SysCoord {
        self.desc.rob_to_sys(
            self.acts.pos()
        )
    }
}

impl<A, D, S> Robot<A, D, S> 
where
    A : PtpActuators,
    D : Descriptor<RobCoord = A::RobCoord>,
    S : Station
{
    pub async fn move_ptp(&mut self, pos_sys : D::SysCoord, f_gen : Factor) -> Result<(), RobotError<A, D, S>> {
        self.acts.move_ptp(
            self.desc.sys_to_rob(pos_sys)
                .map_err(|err| RobotError::<A, D, S>::DescError(err))?, 
            f_gen
        ).await
            .map_err(|err| RobotError::<A, D, S>::ActsError(err))?;
        Ok(())
    }
}