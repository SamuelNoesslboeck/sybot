extern crate alloc;

use core::cell::RefCell;

use alloc::{collections::BTreeMap, rc::Rc};
use glam::{Vec3, Mat3};

// Submodules
    #[cfg(test)]
    mod test;
// 

pub trait Point {
    fn pos<'a>(&'a self) -> &'a Vec3;
    fn ori<'a>(&'a self) -> &'a Mat3;

    fn shift(&mut self, by : Vec3);
    fn transform(&mut self, by : Mat3);

    fn as_wo<'a>(&'a self) -> Option<&'a WorldObj>;
}

#[derive(Clone)]
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
    fn pos<'a>(&'a self) -> &'a Vec3 {
        &self.pos   
    }
    
    fn ori<'a>(&'a self) -> &'a Mat3 {
        &self.ori
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

#[derive(Default, Clone)]
pub struct WorldObj {
    pos : Position,

    pub sub : BTreeMap<String, Rc<RefCell<dyn Point>>>
}

impl AsRef<Position> for WorldObj {
    fn as_ref(&self) -> &Position {
        &self.pos
    }
}

impl Point for WorldObj {
    fn pos<'a>(&'a self) -> &'a Vec3 {
        self.pos.pos()
    }

    fn ori<'a>(&'a self) -> &'a Mat3 {
        self.pos.ori()
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
}

impl WorldObj {
    pub fn new(pos : Position) -> Self {
        Self {
            pos,
            sub: BTreeMap::new()
        }
    }

    pub fn add_point(&mut self, name : String, point : Rc<RefCell<dyn Point>>) {
        if name.contains('/') {
            panic!("Bad point name! Point names must not contain '/'! (Name: {})", name);
        }
        
        self.sub.insert(name, point);
    }

    fn trans_pos_step(&self, split : &[String], index : usize) -> Option<Vec3> {
        if index > split.len() {
            return None;
        }

        if let Some(point) = self.sub.get(&split[index]) {
            let p = point.borrow();

            if split.len() == index {
                return Some((*self.ori()) * (*p.pos()) + *self.pos());
            }

            if let Some(wo) = p.as_wo() {
                if let Some(pos) = wo.trans_pos_step(split, index + 1) {
                    return Some((*self.ori()) * (pos) + *self.pos());
                } 
            } 
        } 

        None
    }

    pub fn trans_pos(&self, path : String) -> Option<Vec3> {
        let path_split : Vec<String> = path.split('/').map(|elem| elem.to_owned()).collect();
        self.trans_pos_step(&path_split, 0)
    }
}
