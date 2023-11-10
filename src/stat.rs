pub trait Station {
    fn home(&mut self) -> Result<(), crate::Error>;
}