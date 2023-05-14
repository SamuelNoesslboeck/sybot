use core::{cell::RefCell, ops::Deref};

use alloc::{rc::Rc, collections::BTreeMap};
use glam::{Vec3, Mat3};
use serde::{Serialize, Deserialize, ser::{SerializeMap, SerializeStruct}, de::Visitor};

use crate::{Position, Point, WorldObj, PointRef};

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PositionDes {
    pub pos : [f32; 3],
    pub ori : [[f32; 3]; 3]
}

impl From<Position> for PositionDes {
    fn from(pos : Position) -> Self {
        PositionDes { 
            pos: pos.pos.to_array(), 
            ori: pos.ori.to_cols_array_2d()
        }
    }
}

impl Into<Position> for PositionDes {
    fn into(self) -> Position {
        Position::new_ori(
            Vec3::from(self.pos),
            Mat3::from_cols_array_2d(&self.ori)
        )
    }
}

impl Serialize for Position {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer {
        let pos = PositionDes::from(self.clone());
        pos.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Position {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> {   
        Ok(PositionDes::deserialize(deserializer)?.into())
    }
}

impl Serialize for dyn Point {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer {
        if let Some(wo) = self.as_wo() {
            wo.serialize(serializer)
        } else if let Some(pos) = self.as_pos() {
            pos.serialize(serializer)
        } else {
            panic!("Unknown implementation of Point! Your point must be either convertable to a Position or a Worldobject!");
        }
    }
}

impl Serialize for PointRef {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer {
        let point = self.borrow();
        point.serialize(serializer)
    }
}

impl<'de> Visitor<'de> for PointRef {
    type Value = Self;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(formatter, "A position or worldobj obj")
    }

    // TODO: Make parsing safer
    fn visit_map<A>(self, map: A) -> Result<Self::Value, A::Error>
        where
            A: serde::de::MapAccess<'de>, {
        if let Some(pos) = map.next_entry::<String, Position>()? {
            let sub = map.next_entry::<String, BTreeMap<String, PointRef>>()?.unwrap();

            return Ok(PointRef(Rc::new(RefCell::new(WorldObj::new(
                pos.1,
                sub.1
            )))))
        } 

        
    }
}

impl<'de> Deserialize<'de> for PointRef {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> {
        if let Ok(wo) = WorldObj::dese {
            return Ok(PointRef(Rc::new(RefCell::new(wo))))
        } 
            
        Ok(PointRef(Rc::new(RefCell::new(Position::deserialize(deserializer)?))))
    }
}