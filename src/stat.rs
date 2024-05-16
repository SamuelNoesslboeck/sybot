use syact::{SyncActuator, SyncActuatorGroup};

use crate::Robot;

pub trait Station<G, T, const C : usize> 
where
    G : SyncActuatorGroup<T, C>,
    T : SyncActuator + ?Sized + 'static
{
    type Robot : Robot<G, T, C>;

    fn home(&mut self, rob : &mut Self::Robot) -> Result<(), crate::Error>;
}