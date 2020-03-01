/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.DriveTrain;

import frc.robot.Constants.DriveTrainConstants;


/**
 * Unfinished, I don't know if we plan to add this at all...
 */
public class DriveUntilDistance extends PIDCommand {
  private double initialHeading = 0; // Not finished, in case we want to not to reset heading.

  /**
   * Creates a new DriveUntilDistance command. This command makes the robot drive a certain distance
   * forward, based on encoder values.
   * 
   * @param distance How far away from the object should the robot stop, in inches
   * @param driveTrain The {@link DriveTrain} subsystem
   */
  public DriveUntilDistance(double distance, DriveTrain driveTrain) {
    super(
        new PIDController(DriveTrainConstants.DRIVE_P, DriveTrainConstants.DRIVE_I, DriveTrainConstants.DRIVE_D),
        driveTrain::getDistanceToObject,
        distance,
        output -> driveTrain.driveStraight(output),
        driveTrain
    );
    getController().setTolerance(DriveTrainConstants.DRIVE_TARGET_TOLERANCE);

    driveTrain.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return getController().atSetpoint()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    getController().close(); // Delete the controller when we are finished
  }
}
