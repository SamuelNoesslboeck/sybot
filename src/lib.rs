#![doc = include_str!("../README.md")]
#![crate_name = "sybot"]
#![allow(async_fn_in_trait)]
// #![deny(missing_docs)]

use core::marker::PhantomData;

use syact::{ActuatorError, DefinedActuator, SyncActuator};
use syunit::{Factor, UnitSet, Unit};

/* Submodules */
    mod acts;

    mod desc;
/**/

/* CORE TRAITS */
pub trait Actuators {
    type RobCoord;
    type Error;

    fn pos(&self) -> Self::RobCoord;

    async fn move_abs(&mut self, pos : Self::RobCoord, f_gen : Factor) -> Result<(), Self::Error>;
}

pub trait PtpActuators : Actuators {
    async fn move_ptp(&mut self, pos : Self::RobCoord, f_gen : Factor) -> Result<(), Self::Error>;
}

pub trait Descriptor {
    type SysCoord;
    type RobCoord;
    type Error;

    fn rob_to_sys(&self, rob : Self::RobCoord) -> Self::SysCoord;

    fn sys_to_rob(&self, sys : Self::SysCoord) -> Result<Self::RobCoord, Self::Error>;
}

pub trait System {
    type Error;
}

pub enum RobotError<A : Actuators, D : Descriptor, S : System> {
    ActsError(A::Error),
    DescError(D::Error),
    StatError(S::Error)
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

    async fn move_abs(&mut self, pos : Self::RobCoord, f_gen : Factor) -> Result<(), Self::Error> {
        let (res_a, res_b) = futures::join!(
            self.a.drive_abs(pos.0, f_gen),
            self.b.drive_abs(pos.1, f_gen)
        );

        // Map the different errors                     
        res_a.map_err(|err| Actuators2Error::ComponentA(err))?;
        res_b.map_err(|err| Actuators2Error::ComponentB(err))?;

        Ok(())
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
    S : System
{
    pub acts : A,
    pub desc : D,
    pub stat : S
}

impl<A, D, S> Robot<A, D, S> 
where
    A : Actuators,
    D : Descriptor<RobCoord = A::RobCoord>,
    S : System
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
    S : System
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