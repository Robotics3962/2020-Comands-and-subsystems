package frc.robot;

/**
 * This file contains the mappings between the joystick buttons 
 * and the command they will run
 */

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

public class JoyStickControl {
  // get both drive and operational joysticks
  public static Joystick driveJoystick = new Joystick(RobotMap.Joystick0Id);
  static Joystick operationJoyStick = new Joystick(RobotMap.Joystick1Id); 
  
  public JoyStickControl(){
    
    // get the buttons on the drive joystick
    JoystickButton driveButtonA = new JoystickButton(driveJoystick, Button.kA.value);
    JoystickButton driveButtonB = new JoystickButton(driveJoystick, Button.kB.value);
    JoystickButton driveButtonX = new JoystickButton(driveJoystick, Button.kX.value);
    JoystickButton driveButtonY = new JoystickButton(driveJoystick, Button.kY.value);
    JoystickButton driveButtonLS = new JoystickButton(driveJoystick, Button.kBumperLeft.value);
    JoystickButton driveButtonRS = new JoystickButton(driveJoystick, Button.kBumperRight.value);
    JoystickButton driveButtonBack = new JoystickButton(driveJoystick, Button.kBack.value);
    JoystickButton driveButtonStart = new JoystickButton(driveJoystick, Button.kStart.value);

    // assign drive joystick buttons to commands
    //driveButtonA.whileHeld(new SpinnerEnableDisplayColorInfoCmd());
    //driveButtonBack.whileHeld(new GrabBallCmd());


    // second joystick I'm calling it operational - no command mapping yet
    JoystickButton opButtonA = new JoystickButton(operationJoyStick, Button.kA.value);
    JoystickButton opButtonB = new JoystickButton(operationJoyStick, Button.kB.value);
    JoystickButton opButtonX = new JoystickButton(operationJoyStick, Button.kX.value);
    JoystickButton opButtonY = new JoystickButton(operationJoyStick, Button.kY.value);
    JoystickButton opButtonLS = new JoystickButton(operationJoyStick, Button.kBumperLeft.value);
    JoystickButton opButtonRS = new JoystickButton(operationJoyStick, Button.kBumperRight.value);
    JoystickButton opButtonBack = new JoystickButton(operationJoyStick, Button.kBack.value);
    JoystickButton opButtonStart = new JoystickButton(operationJoyStick, Button.kStart.value);

    driveButtonA.whileHeld(new ClimberElevateCmd());
    driveButtonB.whileHeld(new ClimberDownCmd());
    driveButtonX.whileHeld(new ShooterSpinFeederCWCmd());
    driveButtonY.whileHeld(new ShooterSpinFeederCCWCmd());
    driveButtonA.whileHeld(new ClimberUpCmd());
    driveButtonB.whileHeld(new ClimberDownCmd());
    //driveButtonBack.whileHeld()

    opButtonLS.whileHeld(new ShooterShootCmd());
    //opButtonLS.whileHeld(new ShooterStartCmd());
    opButtonRS.whileHeld(new ShooterStopCmd());
    opButtonBack.whenPressed(new IntakeRetractCmd());
    opButtonStart.whenPressed(new IntakeDeployCmd());
    opButtonA.whileHeld(new IntakeSpinCmd());
    opButtonB.whenPressed(new IntakeNeutralCmd());
    opButtonX.whileHeld(new LiftRunCmd());//mech calls this uptake
    opButtonY.whileHeld(new LiftRunReverseCmd());//mech calls this uptake
  }
    
  public double getLeftThrottle() {
		return driveJoystick.getY(); 
	}	

	public double getRightRotation() {
        return driveJoystick.getRawAxis(4);
    }

  public static boolean deadManSwitch() {
    boolean active = false;
    active = operationJoyStick.getRawButton(Button.kBumperLeft.value);
    SmartDashboard.putBoolean("deadman switch:", active);
    active = true;
    return active;
  }
}
