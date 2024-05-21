use syact::{SyncActuator, SyncActuatorGroup};

use crate::Robot;

/// A station defines the environment of a stationary robot
#[allow(async_fn_in_trait)]
pub trait Station<G, T, const C : usize> 
where
    G : SyncActuatorGroup<T, C>,
    T : SyncActuator + ?Sized + 'static
{
    /// The robot of the station
    type Robot : Robot<G, T, C>;

    /// Start calibrating the station and robot
    async fn calibrate(&mut self, rob : &mut Self::Robot) -> Result<(), crate::Error>;

    /// Drive to the home position, often includes calling `calibrate()`
    async fn home(&mut self, rob : &mut Self::Robot) -> Result<(), crate::Error>;
}