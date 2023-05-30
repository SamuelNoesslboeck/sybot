extern crate alloc;

use core::{cell::RefCell, fmt::Debug, ops::{Deref, DerefMut}};

use alloc::{collections::BTreeMap, rc::Rc};
use glam::{Vec3, Mat3};
use serde::{Serialize, Deserialize};
// use serde::de::DeserializeOwned;

// Submodules
    mod des;

    pub mod math;
// 

pub type Error = Box<dyn std::error::Error>;

pub trait Point : Debug {
    // Coords
        fn x(&self) -> f32;
        fn y(&self) -> f32;
        fn z(&self) -> f32;
    // 

    fn pos<'a>(&'a self) -> &'a Vec3;
    fn pos_mut<'a>(&'a mut self) -> &'a mut Vec3;

    fn ori<'a>(&'a self) -> &'a Mat3;
    fn ori_mut<'a>(&'a mut self) -> &'a mut Mat3;

    fn shift(&mut self, by : Vec3);
    fn transform(&mut self, by : Mat3);

    fn as_pos<'a>(&'a self) -> Option<&'a Position>;
    fn as_wo<'a>(&'a self) -> Option<&'a WorldObj>;

    fn trans_other(&self, v : Vec3) -> Vec3 {
        (*self.ori()) * v 
    }

    fn shift_other(&self, v : Vec3) -> Vec3 {
        (*self.pos()) + v
    }

    fn to_higher_system(&self, v : Vec3) -> Vec3 {
        self.shift_other(self.trans_other(v))
    }
}

#[derive(Clone, Debug)]
pub struct Position {
    pos : Vec3,
    ori : Mat3
}

impl Default for Position {
    fn default() -> Self {
        Self {
            pos: Vec3::default(),
            ori: Mat3::IDENTITY
        }
    }
}

impl Point for Position {
    // Positions
        fn x(&self) -> f32 {
            self.pos.x
        }

        fn y(&self) -> f32 {
            self.pos.y
        }

        fn z(&self) -> f32 {
            self.pos.z
        }
    // 

    fn pos<'a>(&'a self) -> &'a Vec3 {
        &self.pos   
    }

    fn pos_mut<'a>(&'a mut self) -> &'a mut Vec3 {
        &mut self.pos
    }
    
    fn ori<'a>(&'a self) -> &'a Mat3 {
        &self.ori
    }

    fn ori_mut<'a>(&'a mut self) -> &'a mut Mat3 {
        &mut self.ori
    }

    fn shift(&mut self, by : Vec3) {
        self.pos += by;
    }

    fn transform(&mut self, by : Mat3) {
        self.pos = by * self.pos;
    }

    fn as_wo<'a>(&'a self) -> Option<&'a WorldObj> {
        None
    }

    fn as_pos<'a>(&'a self) -> Option<&'a Position> {
        Some(self)
    }
}

impl Position {
    pub fn new(pos : Vec3) -> Self {
        Self {
            pos, 
            ori: Mat3::IDENTITY
        }
    }

    pub fn new_ori(pos : Vec3, ori : Mat3) -> Self {
        Self { pos, ori }
    }

    pub fn to_wo(self) -> WorldObj {
        WorldObj::new(self)
    }
}

#[derive(Clone, Debug)]
pub struct PointRef(pub Rc<RefCell<dyn Point>>);

impl Deref for PointRef {
    type Target = Rc<RefCell<dyn Point>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for PointRef {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl PointRef {
    pub fn pos(&self) -> Vec3 {
        *self.borrow().pos()
    }

    pub fn clone_no_ref(&self) -> PointRef {
        let p = self.borrow(); 

        if let Some(wo) = p.as_wo() {
            PointRef(Rc::new(RefCell::new(wo.clone())))
        } else if let Some(pos) = p.as_pos() { 
            PointRef(Rc::new(RefCell::new(pos.clone())))
        } else {
            panic!("Bad implementation of trait 'Point'!");
        }
    }
}

#[derive(Default, Clone, Debug, Serialize, Deserialize)]
pub struct WorldObj {
    pos : Position,
    pub sub : BTreeMap<String, PointRef>
}

impl AsRef<Position> for WorldObj {
    fn as_ref(&self) -> &Position {
        &self.pos
    }
}

impl Point for WorldObj {
        // Positions
        fn x(&self) -> f32 {
            self.pos.x()
        }

        fn y(&self) -> f32 {
            self.pos.y()
        }

        fn z(&self) -> f32 {
            self.pos.z()
        }
    // 

    fn pos<'a>(&'a self) -> &'a Vec3 {
        self.pos.pos()
    }

    fn pos_mut<'a>(&'a mut self) -> &'a mut Vec3 {
        &mut self.pos.pos
    }

    fn ori<'a>(&'a self) -> &'a Mat3 {
        self.pos.ori()
    }

    fn ori_mut<'a>(&'a mut self) -> &'a mut Mat3 {
        &mut self.pos.ori
    }

    fn shift(&mut self, by : Vec3) {
        self.pos.shift(by)
    }

    fn transform(&mut self, by : Mat3) {
        self.pos.transform(by)
    }

    fn as_wo<'a>(&'a self) -> Option<&'a WorldObj> {
        Some(self)
    }

    fn as_pos<'a>(&'a self) -> Option<&'a Position> {
        Some(&self.pos)
    }
}

impl WorldObj {
    pub fn new(pos : Position) -> Self {
        Self {
            pos,
            sub: BTreeMap::new()
        }
    }

    pub fn new_sub(pos : Position, sub : BTreeMap<String, PointRef>) -> Self {
        Self {
            pos, sub
        }
    }

    pub fn add_point(&mut self, name : String, point : PointRef) {
        if name.contains('/') {
            panic!("Bad point name! Point names must not contain '/'! (Name: {})", name);
        }
        
        self.sub.insert(name, point);
    }

    pub fn point<S : Into<String>>(&self, path : S) -> Option<PointRef> {
        let path_s = path.into();
        let path_split : Vec<&str> = path_s.split('/').collect();
        self.point_path(&path_split)
    }

    pub fn req_point<S : Into<String>>(&self, path : S) -> Result<PointRef, crate::Error> {
        let path_s = path.into();
        if let Some(p) = self.point(path_s.clone()) {
            Ok(p)
        } else {
            Err(format!("The system requires a point with path '{}'", &path_s).into())
        }
    }

    pub fn point_path(&self, path : &[&str]) -> Option<PointRef> {
        self.resolve_path_step(path, 0)
    }

    pub fn req_point_path(&self, path : &[&str]) -> Result<PointRef, crate::Error> {
        if let Some(p) = self.point_path(path) {
            Ok(p)
        } else {
            Err(format!("The system requires a point with path ({:?})", path).into())
        }
    }

    fn resolve_path_step(&self, split : &[&str], mut index : usize) -> Option<PointRef> {
        if index > split.len() {
            return None;
        }

        if let Some(point) = self.sub.get(split[index]) {
            let p = point.borrow();

            index += 1;
            
            if split.len() == index {
                return Some(point.clone());
            }

            if let Some(wo) = p.as_wo() {
                return wo.resolve_path_step(split, index);
            } 
        } 

        None
    }

    // fn trans_pos_step(&self, split : &[String], index : usize) -> Option<Vec3> {
    //     if index > split.len() {
    //         return None;
    //     }

    //     if let Some(point) = self.sub.get(&split[index]) {
    //         let p = point.borrow();

    //         if split.len() == index {
    //             return Some((*self.ori()) * (*p.pos()) + *self.pos());
    //         }

    //         if let Some(wo) = p.as_wo() {
    //             if let Some(pos) = wo.trans_pos_step(split, index + 1) {
    //                 return Some((*self.ori()) * (pos) + *self.pos());
    //             } 
    //         } 
    //     } 

    //     None
    // }

    // pub fn trans_pos(&self, path : String) -> Option<Vec3> {
    //     let path_split : Vec<String> = path.split('/').map(|elem| elem.to_owned()).collect();
    //     self.trans_pos_step(&path_split, 0)
    // }
}
