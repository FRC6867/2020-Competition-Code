/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.DriveTrain;

import frc.robot.Constants.DriveTrainConstants;

public class DriveForward extends PIDCommand {
  /**
   * Creates a new DriveForward command.
   * 
   * @param distance The distance to drive, in inches
   * @param driveTrain The {@link DriveTrain} subsystem
   */
  public DriveForward(double distance, DriveTrain driveTrain) {
    super(
      new PIDController(DriveTrainConstants.DRIVE_P, DriveTrainConstants.DRIVE_I, DriveTrainConstants.DRIVE_D),
      driveTrain::getDistanceDriven,
      distance,
      output -> driveTrain.driveStraight(output),
      driveTrain
    );

    driveTrain.reset(); // Make sure we are in relative position
    setPID();
    getController().setTolerance(DriveTrainConstants.DRIVE_TARGET_TOLERANCE); // Should be one unit, inches in this case

    addRequirements(driveTrain);
  }

  private void setPID() {
    getController().setPID(
      SmartDashboard.getNumber("Drive P", getController().getP()),
      SmartDashboard.getNumber("Drive I", getController().getI()),
      SmartDashboard.getNumber("Drive D", getController().getD())
    );

    SmartDashboard.putNumber("Drive P", getController().getP());
    SmartDashboard.putNumber("Drive I", getController().getI());
    SmartDashboard.putNumber("Drive D", getController().getD());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return getController().atSetpoint(); // Disabled for tuning
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    getController().close(); // Delete the controller when we are finished
  }
}
