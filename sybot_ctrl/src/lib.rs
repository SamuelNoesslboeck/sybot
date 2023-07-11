/// Direction to be moved to
pub enum Direction {
    XPos,
    XNeg,
    YPos,
    YNeg,
    ZPos,
    ZNegs
}

/// Index of axis to be moved
pub type Axis = usize;

pub enum Type {
    Joint(Axis),
    Lin(Direction)
}
