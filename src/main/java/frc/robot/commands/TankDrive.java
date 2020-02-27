/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import frc.robot.Constants.DriveTrainConstants;

/**
 * Rawr I have a tank
 */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private final DoubleSupplier m_rightStick;
  private final DoubleSupplier m_leftStick;

  private double m_lastRightSpeed = 0;
  private double m_lastLeftSpeed = 0;

  /**
   * Creates a new TankDrive command.
   * 
   * @param rightStick Right Y-axis DoubleSupplier
   * @param leftStick Left Y-axis DoubleSupplier
   * @param driveTrain {@link DriveTrain} subsystem
   */
  public TankDrive(DoubleSupplier rightStick, DoubleSupplier leftStick, DriveTrain driveTrain) {
    m_driveTrain = driveTrain;

    m_rightStick = rightStick;
    m_leftStick = leftStick;
    
    addRequirements(m_driveTrain);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double rightSpeed = getSpeedWithMin(m_rightStick.getAsDouble(), m_lastRightSpeed);
    final double leftSpeed = getSpeedWithMin(m_leftStick.getAsDouble(), m_lastLeftSpeed);

    m_driveTrain.drive(
      rightSpeed,
      leftSpeed
    );

    m_lastRightSpeed = rightSpeed;
    m_lastLeftSpeed = leftSpeed;
  }

  private double getSpeedWithMin(double speed, double lastSpeed) {
    if (Math.abs(speed) > DriveTrainConstants.MIN_SPEED_THRESHOLD) {
      return speed;
    } else { // Going to 0 power
      return getArtificialCoast(lastSpeed);
    }
  }

  private double getArtificialCoast(double lastSpeed) {
    final double artificialCoastSpeed = lastSpeed / DriveTrainConstants.ARTIFICIAL_COAST_NUM;

    if (Math.abs(artificialCoastSpeed) > DriveTrainConstants.MIN_SPEED_THRESHOLD) {
      return artificialCoastSpeed;
    } else {
      return 0;
    }
  }

  // Default command - never stop
  @Override
  public boolean isFinished() {
    return false;
  }

  // Stop motor when command ends
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0);
  }
}
