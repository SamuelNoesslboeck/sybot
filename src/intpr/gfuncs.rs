use std::process::exit;

use stepper_lib::units::*;

use crate::intpr::gcode::*;
use crate::robot::SafeRobot; 

// General functions
    /// G0 X{Position} Y{Position} Z{Position} DECO{Angle} \
    /// Rapid positioning
    pub fn g0<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, c : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let pos = robot.safe_pos(
            arg_by_letter(args, 'X'), 
            arg_by_letter(args, 'Y'), 
            arg_by_letter(args, 'Z')
        );

        let phis = robot.safe_phis_for_vec(pos,
            args_by_letter(args, 'D').as_slice()
        )?;

        let f_bend = arg_by_letter(args, 'B').unwrap_or(1.0);
        let f_speed = arg_by_letter(args, 'S').unwrap_or(1.0);

        robot.apply_bend_f(f_bend);
        robot.apply_speed_f(f_speed);

        let deltas = if c.minor_number() == 0 {
            robot.move_j_abs(robot.gammas_from_phis(phis))?
        } else if c.minor_number() == 1 {
            robot.move_j_abs_async(robot.gammas_from_phis(phis))?;
            robot.await_inactive()?
        } else {
            // Create error!
            panic!("Bad minor number!");
        };

        robot.update(None)?;

        robot.apply_bend_f(1.0);
        robot.apply_speed_f(1.0);
        
        Ok(serde_json::json!({ 
            "points": pos.to_array(), 
            "phis": Vec::from(phis),
            "deltas": Vec::from(deltas)
        }))
    }

    /// G4 X{Seconds} P{Milliseconds}
    /// Dwell (sleeping)
    pub fn g4<R : SafeRobot<C>, const C : usize>
        (_ : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let seconds = 
            arg_by_letter(args, 'X').unwrap_or(0.0)            // Seconds
            + arg_by_letter(args, 'P').unwrap_or(0.0)/1000.0;  // Milliseconds
        std::thread::sleep(core::time::Duration::from_secs_f32(seconds));
        Ok(serde_json::json!(seconds))
    }

    /// G8 X{Position} Y{Position} Z{Position} DECO{Angle} \
    /// Rapid positioning async
    pub fn g8<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let angles = robot.safe_phis_for_vec(robot.safe_pos(
                arg_by_letter(args, 'X'), 
                arg_by_letter(args, 'Y'), 
                arg_by_letter(args, 'Z')
            ),
            vec![0.0; C].as_slice()
        )?; 

        robot.move_j_abs_async(robot.gammas_from_phis(angles))?;
        robot.await_inactive()?;
        robot.update(None)?;

        Ok(serde_json::json!(Vec::from(angles)))
    }   

        /// G28 \
    /// Return to home position
    pub fn g28<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error>  
    {
        match robot.measure() {
            Ok(deltas) => Ok(serde_json::json!(Vec::from(deltas))),
            Err(meas) => {
                println!(" -> Problems with measurement! {:?}", meas);      // TODO: Add proper error
                Ok(serde_json::Value::Null)
            }
        }
    }

    // /// G29 \
    // /// Return to home position async
    // pub fn g29<R : SafeRobot<C>, const C : usize>
    //     (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    // {
    //     // arm.measure(2);
    //     robot.measure_async(2);
    //     robot.await_inactive();
    //     robot.update(None);
    //     let home = *robot.home_pos();
    //     robot.set_end(&home);
    //     Ok(serde_json::Value::Null)
    // }

    // Extra functions
    pub fn g100<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let phis = robot.safe_phis(args_by_iterate_fixed::<C>(args, 'A'))?;

        let deltas = robot.move_j_abs(robot.gammas_from_phis(phis))?;
        robot.update(None)?;
        Ok(serde_json::json!({ 
            "phis": Vec::from(phis),
            "deltas": Vec::from(deltas)
        }))
    }

    // Debug
    pub fn g1000<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        Ok(serde_json::json!({ 
            "phis": Vec::from(robot.phis()),
            "gammas": Vec::from(robot.gammas()),
            "pos": robot.pos().to_array()
        }))
    }

    pub fn g1100<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let phis = robot.safe_phis(args_by_iterate_fixed::<C>(args, 'A'))?;

        robot.write_phis(&phis);
        robot.update(None)?;
        Ok(serde_json::json!(Vec::from(phis)))
    }
//

// Misc Functions
    pub fn m3<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        // TODO: Add response
        robot.activate_tool();
        robot.activate_spindle(true);

        Ok(serde_json::Value::Null)
    }

    pub fn m4<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        Ok(serde_json::json!(robot.activate_spindle(false)))
    }

    pub fn m5<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        Ok(serde_json::json!(robot.deactivate_tool()))
    }

    pub fn m30<R : SafeRobot<C>, const C : usize>
    (_ : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        println!("Program finished!");
        exit(0);
    }

    // Additional functions
    pub fn m119<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, args : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        let gamma_opt = robot.gamma_tool();

        if let Some(gamma) = gamma_opt {
            return Ok(serde_json::json!(robot.rotate_tool_abs(Gamma(arg_by_letter(args, 'A').unwrap_or(gamma.0)))));
        }
        
        Ok(serde_json::Value::Null)
    }

    // Debug functions
    pub fn m1006<R : SafeRobot<C>, const C : usize>
        (robot : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        Ok(serde_json::to_value(robot.get_tool().unwrap().get_json()).unwrap())
        // Ok(serde_json::to_value(
        //     robot.get_tools().iter().map(
        //         |t| t.get_json()
        //     ).collect::<Vec<serde_json::Value>>()
        // ).unwrap())
    }
// 

// Programm functions
    pub fn o0<R : SafeRobot<C>, const C : usize>
        (_ : &mut R, _ : &GCode, _ : &Args) -> Result<serde_json::Value, crate::Error> 
    {
        println!("test");
        Ok(serde_json::Value::Null)
    }
//

// Tool
pub fn t<R : SafeRobot<C>, const C : usize>
    (robot : &mut R, index : usize) -> Result<serde_json::Value, crate::Error> 
{
    if let Some(tool) = robot.set_tool_id(index) {
        return Ok(tool.get_json())
    }

    Ok(serde_json::Value::Null)
    // Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, " -> No tool has been found for this index!"))
}
