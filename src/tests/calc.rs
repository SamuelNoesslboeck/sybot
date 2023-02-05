use super::*;

mod postion
{
    use super::*;

    #[test]
    fn double_convert() {
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        );

        let pos = Vec3::new(0.0, 350.0, 400.0);
        
        let angles = syarm.get_with_fixed_dec(pos, 0.0);
        let points = syarm.points_by_phis(&angles);

        assert!((pos - points[3]).length() > f32::EPSILON);
    }

    #[test]
    fn single_convert() {
        let syarm = SyArm::from_conf(
            JsonConfig::read_from_file("res/SyArm_Mk1.conf.json")
        );
        
        let angles = [ 0.0, PI / 2.0, -PI / 2.0, 0.0 ];
        let points = syarm.points_by_phis(&angles);

        dbg!(points);
    }

    
}