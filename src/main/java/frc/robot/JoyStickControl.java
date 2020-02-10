package frc.robot;

/**
 * This file contains the mappings between the joystick buttons 
 * and the command they will run
 */

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

class JoystickControl {
  // get both drive and operational joysticks
  Joystick driveJoystick = null;
  Joystick operationJoyStick = null; 
  
  public JoystickControl(){
    Joystick driveJoystick = new Joystick(RobotMap.Joystick0Id);
    Joystick operationJoyStick = new Joystick(RobotMap.Joystick1Id);
    
    // get the buttons on the drive joystick
    JoystickButton driveButtonA = new JoystickButton(driveJoystick, RobotMap.JoystickButtonA);
    JoystickButton driveButtonB = new JoystickButton(driveJoystick, RobotMap.JoystickButtonB);
    JoystickButton driveButtonX = new JoystickButton(driveJoystick, RobotMap.JoystickButtonX);
    JoystickButton driveButtonY = new JoystickButton(driveJoystick, RobotMap.JoystickButtonY);
    JoystickButton driveButtonLS = new JoystickButton(driveJoystick, RobotMap.JoystickButtonShoulderLeft);
    JoystickButton driveButtonRS = new JoystickButton(driveJoystick, RobotMap.JoystickButtonShoulderRight);
    JoystickButton driveButtonBack = new JoystickButton(driveJoystick, RobotMap.JoystickButtonBack);
    JoystickButton driveButtonStart = new JoystickButton(driveJoystick, RobotMap.JoystickButtonStart);

    // assign drive joystick buttons to commands
    //driveButtonA.whileHeld(new DumpInfoCmd());
    //driveButtonBack.whileHeld(new GrabBallCmd());


    // second joystick I'm calling it operational - no command mapping yet
    JoystickButton opButtonA = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonA);
    JoystickButton opButtonB = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonB);
    JoystickButton opButtonX = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonX);
    JoystickButton opButtonY = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonY);
    JoystickButton opButtonLS = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonShoulderLeft);
    JoystickButton opButtonRS = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonShoulderRight);
    JoystickButton opButtonBack = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonBack);
    JoystickButton opButtonStart = new JoystickButton(operationJoyStick, RobotMap.JoystickButtonStart);

    //opButtonA.whenPressed(new LockWristCmd());
    //opButtonLS.whileHeld(new ElevatorDownCmd());
  }
    
  public double getLeftThrottle() {
		return driveJoystick.getY(); 
	}	

	public double getRightRotation() {
        return driveJoystick.getRawAxis(4);
    }
}
