use core::cell::RefCell;

use alloc::rc::Rc;
use glam::Vec3;

use crate::{ Position, PointRef};

#[test]
fn parsing_rcs() -> Result<(), Box<dyn std::error::Error>> {
    let j_str =     // include_str!("../assets/rcs.json");
                    include_str!("assets/pos.json");
    let wo : PointRef = serde_json::from_str(j_str)?;

    dbg!(wo);

    Ok(())
}

#[test]
fn serial_rcs() -> Result<(), Box<dyn std::error::Error>> {
    let point = Position::new(Vec3::X);
    let poi_ref = PointRef(Rc::new(RefCell::new(point)));

    let j = serde_json::to_value(poi_ref)?;
    let j_str = serde_json::to_string(&j)?;

    let _ : PointRef = serde_json::from_value(j.clone())?;
    let _ : PointRef = serde_json::from_str(&j_str)?; 

    Ok(())
}