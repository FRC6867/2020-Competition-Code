/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import frc.robot.Constants.ControllerConstants;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Deprecated in favor of {@link TankDrive}, currently does not work
 */
public class GTADrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain m_driveTrain;
  private final Joystick m_gamepad;

  /**
   * Creates a new Drive command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GTADrive(DriveTrain driveTrain, Joystick joystick) {
    m_driveTrain = driveTrain;
    m_gamepad = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Right trigger -> forwards
    if (m_gamepad.getRawAxis(ControllerConstants.KRAB_LEFT_BUTTON) >= 0.1)
    {
      m_driveTrain.driveLeft((m_gamepad.getRawAxis(ControllerConstants.KRAB_LEFT_BUTTON) + getRightXAxis()));
      m_driveTrain.driveRight((m_gamepad.getRawAxis(ControllerConstants.KRAB_LEFT_BUTTON) - getRightXAxis()));
    }

    // Left trigger -> backwards
    else if (m_gamepad.getRawAxis(ControllerConstants.KRAB_RIGHT_BUTTON) >= 0.1)
    {
      m_driveTrain.driveLeft(-(m_gamepad.getRawAxis(ControllerConstants.KRAB_RIGHT_BUTTON) + getRightXAxis()));
      m_driveTrain.driveRight(-(m_gamepad.getRawAxis(ControllerConstants.KRAB_RIGHT_BUTTON) - getRightXAxis()));
    }

    else
    {
      if (getRightXAxis() >= 0.1 || getRightXAxis() <= -0.1)
      {
        m_driveTrain.driveLeft(-getRightXAxis());
        m_driveTrain.driveRight(getRightXAxis());
      }
      else
      {
        m_driveTrain.driveLeft(0);
        m_driveTrain.driveRight(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public double getRightXAxis() {
    return (m_gamepad.getRawAxis(ControllerConstants.RIGHT_STICK_X_AXIS));
  }
}
