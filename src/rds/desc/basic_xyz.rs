use syact::{SyncComp, SyncCompGroup};
use syact::units::*;

use crate::{Descriptor, EmptyConf, AxisConf, Robot};
use crate::rcs::{Position, WorldObj, Point, PointRef};

pub struct BasicXYZDescriptor {
    _aconf : EmptyConf,
    _wobj : WorldObj,

    _tcp : PointRef
}

impl Descriptor<3> for BasicXYZDescriptor {
    // Axis conf
        fn aconf<'a>(&'a self) -> &'a dyn AxisConf {
            &self._aconf
        }

        fn aconf_mut<'a>(&'a mut self) -> &'a mut dyn AxisConf {
            &mut self._aconf
        }
    //

    // Events
        fn update<R : Robot<3, Comp = S, CompGroup = G>, S : SyncComp + ?Sized + 'static, G : SyncCompGroup<S, 3>>(&mut self, _rob : &mut R, _phis : &[Phi; 3]) -> Result<(), crate::Error> {
            Ok(())
        }
    // 

    // Calculation
        fn convert_pos<R : Robot<3, Comp = S, CompGroup = G>, S : SyncComp + ?Sized, G>(&mut self, _rob : &mut R, pos : Position) -> Result<[Phi; 3], crate::Error> {
            Ok([ 
                Phi(pos.x()),
                Phi(pos.y()),
                Phi(pos.z())
            ])
        }
    //

    // World object
        fn wobj<'a>(&'a self) -> &'a WorldObj {
            &self._wobj
        }

        fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj {
            &mut self._wobj
        }

        fn tcp(&self) -> &PointRef {
            &self._tcp
        }
    // 
}