/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.ComponentIDConstants;
import frc.robot.Constants.SpeedConstants;


public class Krab extends SubsystemBase {
  private final TalonSRX KRAB_SLIDE_MOTOR = new TalonSRX(ComponentIDConstants.KRAB_SLIDE_MOTOR_CAN);

  /**
   * Creates a new Krab subsystem.
   */
  public Krab() {

  }

  /**
   * Slides Krab to either side based on sign.
   * Does not handle speed mins.
   * Krab limits are handled.
   * 
   * @param speed Speed and direction to move Krab. Positive - right, negative - left,
   * less than 10% power is ignored
   */
  public void slide(double speed) { // We need to separate left and right because of limits
    if (speed < 0)
    {
      slideLeft(speed); // Negative goes left
    }
    else
    {
      slideRight(speed); // Positive goes right
    }
  }

  private void slideRight(double speed) {
    double limit = SpeedConstants.KRAB_RIGHT_STOP + speed * 1500; // Faster speed - stricter limit.

    if (getPosition() > limit) {
      KRAB_SLIDE_MOTOR.set(ControlMode.PercentOutput, -speed * SpeedConstants.KRAB_SPEED);
    } else {
      stopSlide();
    }
  }

  private void slideLeft(double speed) {
    double limit = SpeedConstants.KRAB_LEFT_STOP - speed * 1500; // Same thing again.

    if (getPosition() < limit) {
      KRAB_SLIDE_MOTOR.set(ControlMode.PercentOutput, -speed * SpeedConstants.KRAB_SPEED);
    } else {
      stopSlide();
    }
  }

  public void stopSlide() {
    KRAB_SLIDE_MOTOR.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return Current Krab position in ticks
   */
  public double getPosition() {
    return KRAB_SLIDE_MOTOR.getSensorCollection().getQuadraturePosition();
  }

  @Override
  public void periodic() {
    // TODO: Add logging methods
  }
}
