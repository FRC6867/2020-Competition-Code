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

import frc.robot.Constants.PIDConstants;

public class DriveForward extends PIDCommand {
  /**
   * Creates a new DriveForward command.
   * 
   * @param distance The distance to drive, in inches
   * @param driveTrain The {@link DriveTrain} subsystem
   */
  public DriveForward(double distance, DriveTrain driveTrain) {
    super(
      new PIDController(PIDConstants.DRIVE_FOR_P, PIDConstants.DRIVE_FOR_I, PIDConstants.DRIVE_FOR_D),
      driveTrain::getDistanceDriven,
      distance,
      output -> driveTrain.driveStraight(output),
      driveTrain
    );

    driveTrain.reset(); // Make sure we are in relative position
    setPID();
    getController().setTolerance(1); // Should be one unit, inches in this case

    addRequirements(driveTrain);
  }

  private void setPID() {
    SmartDashboard.putNumber("Forward P", getController().getP());
    SmartDashboard.putNumber("Forward I", getController().getI());
    SmartDashboard.putNumber("Forward D", getController().getD());
    
    getController().setPID(
      SmartDashboard.getNumber("Forward P", PIDConstants.DRIVE_FOR_P),
      SmartDashboard.getNumber("Forward I", PIDConstants.DRIVE_FOR_P),
      SmartDashboard.getNumber("Forward D", PIDConstants.DRIVE_FOR_P)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return getController().atSetpoint(); // Disabled for tuning
    return false;
  }
}
