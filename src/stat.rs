use syact::{SyncActuator, SyncActuatorGroup};

use crate::Robot;

#[allow(async_fn_in_trait)]
pub trait Station<G, T, const C : usize> 
where
    G : SyncActuatorGroup<T, C>,
    T : SyncActuator + ?Sized + 'static
{
    type Robot : Robot<G, T, C>;

    async fn calibrate(&mut self, rob : &mut Self::Robot) -> Result<(), crate::Error>;

    async fn home(&mut self, rob : &mut Self::Robot) -> Result<(), crate::Error>;
}