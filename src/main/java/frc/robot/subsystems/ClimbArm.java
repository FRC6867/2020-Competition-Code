/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.ComponentIDConstants;


public class ClimbArm extends SubsystemBase {
  //private final VictorSPX CLIMB_MOTOR = new VictorSPX(CANConstants.CLIMB_MOTOR_CAN);
  private final TalonSRX CLIMB_MOTOR = new TalonSRX(ComponentIDConstants.CLIMB_MOTOR_CAN);

  /**
   * Creates a new ClimbArm.
   */
  public ClimbArm() {

  }

  /**
   * Sets motor to @param speed.
   */
  public void climb(double speed) {
    CLIMB_MOTOR.set(ControlMode.PercentOutput, speed);
  }
}