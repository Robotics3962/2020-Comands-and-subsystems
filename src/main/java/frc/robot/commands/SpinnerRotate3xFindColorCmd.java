/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SpinnerStopCmd;

public class SpinnerRotate3xFindColorCmd extends SequentialCommandGroup {
  /**
   * Creates a new SpinnerRotate3xFindColorCmd.
   */
  public SpinnerRotate3xFindColorCmd(String color) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new SpinnerMove1TransitionCmd(26), new SpinnerStopCmd(2), new SpinnerSeekColorCmd(color));
  }
}
