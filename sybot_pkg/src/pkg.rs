#[derive(Debug, Default)]
pub struct RobotPackage {

}

#[derive(Debug, Default)]
pub struct DescPackage {

}

#[derive(Debug, Default)]
pub struct StationPackage {

}

#[derive(Debug, Default)]
pub struct Package {
    info : u32, 
    rob : RobotPackage,
    desc : DescPackage,
    stat : StationPackage
}

impl Package {
    pub fn unpack<R : TryFrom<RobotPackage>, D : TryFrom<DescPackage>, S : TryFrom<StationPackage>>(self) -> Result<(R, D, S), crate::Error> {

    }
}