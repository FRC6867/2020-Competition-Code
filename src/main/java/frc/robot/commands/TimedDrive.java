/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TimedDrive extends SequentialCommandGroup {
  /**
   * Creates a new TimedDrive.
   */
  public TimedDrive(double time, double speed, DriveTrain driveTrain) {
    super(
      new InstantCommand(() -> driveTrain.drive(speed, speed), driveTrain),
      new WaitCommand(time),
      new InstantCommand(() -> driveTrain.drive(0, 0), driveTrain)
    );
  }
}
