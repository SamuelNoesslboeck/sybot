use syact::{SyncComp, SyncCompGroup};

use crate::Robot;

pub trait Station<G : SyncCompGroup<T, C>, T : SyncComp + ?Sized + 'static, const C : usize> {
    fn home(&mut self, rob : &mut impl Robot<G, T, C>) -> Result<(), crate::Error>;
}