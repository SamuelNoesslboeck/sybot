use core::cell::RefCell;
use std::collections::HashMap;

use alloc::rc::Rc;
use glam::{Vec3, Mat3};
use serde::{Serialize, Deserialize};

use crate::rcs::{Position, WorldObj, PointRef, Point};

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PositionDes {
    pub pos : [f32; 3],
    pub ori : Option<[[f32; 3]; 3]>
}

impl From<Position> for PositionDes {
    fn from(pos : Position) -> Self {
        PositionDes { 
            pos: pos.pos.to_array(), 
            ori: Some(pos.ori.to_cols_array_2d())
        }
    }
}

impl Into<Position> for PositionDes {
    fn into(self) -> Position {
        if let Some(ori) = &self.ori {
            Position::new_ori(
                Vec3::from(self.pos),
                Mat3::from_cols_array_2d(ori)
            )
        } else {
            Position::new(
                Vec3::from(self.pos)
            )
        }
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

#[derive(Serialize, Deserialize)]
#[serde(untagged)]
enum PointEnum {
    Pos { ori : Option<[[f32; 3]; 3]>, pos : [f32; 3] }, 
    Wo { pos : Position, sub : HashMap<String, PointRef> },
    WoDir { pos: [f32; 3], sub : HashMap<String, PointRef> }
}

impl From<PointRef> for PointEnum {
    fn from(value: PointRef) -> Self {
        let poi = value.borrow();

        if let Some(wo) = poi.as_wo() {
            if wo.pos.ori == Mat3::IDENTITY {
                Self::WoDir { pos: wo.pos.pos().to_array(), sub: wo.sub.clone() }
            } else {
                Self::Wo { pos: wo.pos.clone(), sub: wo.sub.clone() }
            }
        } else if let Some(pos) = poi.as_pos() {
            if pos.ori == Mat3::IDENTITY {
                Self::Pos { pos: pos.pos.to_array(), ori: None }
            } else {
                Self::Pos { ori: Some(pos.ori.to_cols_array_2d()), pos: pos.pos.to_array() }
            }
        } else {
            panic!("Bad implementation of the trait 'Point'!")
        }
    }
}

impl Serialize for PointRef {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer {
        PointEnum::from(self.clone()).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for PointRef {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de> {
        let point = PointEnum::deserialize(deserializer)?;
        
        Ok(match point {
            PointEnum::Pos { ori, pos } => 
                PointRef(Rc::new(RefCell::new(Position::new_ori(Vec3::from(pos), if let Some(o) = &ori {
                    Mat3::from_cols_array_2d(o)
                } else {
                    Mat3::IDENTITY
                })))),
            PointEnum::Wo { pos, sub } => 
                PointRef(Rc::new(RefCell::new(WorldObj::new_sub(pos, sub)))),
            PointEnum::WoDir { pos, sub } => 
                PointRef(Rc::new(RefCell::new(WorldObj::new_sub(Position::new(Vec3::from(pos)), sub))))
        })
    }
}