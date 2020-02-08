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

/**
 * Rawr I have a tank
 */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private final DoubleSupplier m_rightStick;
  private final DoubleSupplier m_leftStick;

  /**
   * Creates a new TankDrive command.
   * 
   * @param rightStick Right Y-axis DoubleSupplier
   * @param leftStick Left Y-axis DoubleSupplier
   * @param driveTrain {@link DriveTrain sub}system
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
    m_driveTrain.drive(
      getSpeedWithMin(m_rightStick.getAsDouble()),
      getSpeedWithMin(m_leftStick.getAsDouble())
    );
  }

  private double getSpeedWithMin(double speed) {
    if (Math.abs(speed) > 0.1) {
      return speed;
    } else { // Don't do anything with less than 10% power
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
