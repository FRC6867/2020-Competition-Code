/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Spark;

import frc.robot.Constants.DecorConstants;

/**
 * Handles fun stuff such as LEDs.
 */
public class Decoration extends SubsystemBase {
  private final Spark m_LEDController = new Spark(DecorConstants.LED_PIN_ID);

  public static double globalLEDState;

  /**
   * Creates a new Decoration subsystem.
   */
  public Decoration() {
    SmartDashboard.putNumber("LED PWM", globalLEDState);
  }

  @Override
  public void periodic() {
    // LED Control
    m_LEDController.set(SmartDashboard.getNumber("LED PWM", DecorConstants.DEFAULT_LED_STATE));
  }
}
