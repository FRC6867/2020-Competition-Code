/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Non-PID based drive forward command.
 */
public class DriveForwardBasic extends CommandBase {
  private final DriveTrain m_driveTrain;

  private final double m_targetDistance;

  /**
   * Creates a new DriveForwardBasic.
   * 
   * @param targetDistance The distance the robot is to drive, in inches
   * @param driveTrain The robot's drive train subsystem
   */
  public DriveForwardBasic(double targetDistance, DriveTrain driveTrain) {
    m_targetDistance = targetDistance;
    m_driveTrain = driveTrain;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.drive(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.getDistanceDriven() >= m_targetDistance;
  }
}
