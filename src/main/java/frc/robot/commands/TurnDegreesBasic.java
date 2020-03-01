/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.lang.Math;

import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesBasic extends CommandBase {
  private final DriveTrain m_driveTrain;

  private final double m_targetDegrees;

  /**
   * Creates a new TurnDegreesBasic.
   */
  public TurnDegreesBasic(double targetDegrees, DriveTrain driveTrain) {
    m_targetDegrees = targetDegrees;
    m_driveTrain = driveTrain;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final double sign = Math.signum(m_targetDegrees);
    m_driveTrain.drive(sign * -0.5, sign * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getHeading()) >= Math.abs(m_targetDegrees);
  }
}
