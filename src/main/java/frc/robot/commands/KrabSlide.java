/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Krab;

public class KrabSlide extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Krab m_krab;
  private final DoubleSupplier m_rightTrigger;
  private final DoubleSupplier m_leftTrigger;

  /**
   * Creates a new KrabSlide command.
   * 
   * @param rightTrigger Right trigger button
   * @param leftTrigger Left trigger button
   * @param Krab Krab subsystem
   */
  public KrabSlide(DoubleSupplier rightTrigger, DoubleSupplier leftTrigger, Krab krab) {
    m_krab = krab;
    m_rightTrigger = rightTrigger;
    m_leftTrigger = leftTrigger;

    addRequirements(m_krab);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_krab.slide(
      getSpeedWithMin(m_rightTrigger.getAsDouble()) -
      getSpeedWithMin(m_leftTrigger.getAsDouble())
    );
  }

  private double getSpeedWithMin(double speed) {
    if (Math.abs(speed) > 0.1) {
      return speed;
    } else { // Don't do anything with less than 10% power
      return 0;
    }
  }

  // Default command - never stop.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Stop motor when command stops.
  @Override
  public void end(boolean interrupted) {
    m_krab.stopSlide();
  }
}
