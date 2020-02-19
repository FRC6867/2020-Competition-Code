/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.IntakeConstants;

public class Intake extends PIDSubsystem {
  private final TalonSRX m_moverMotor = new TalonSRX(IntakeConstants.INTAKE_MOVER_MOTOR_CAN);
  private final VictorSPX m_collectorMotor = new VictorSPX(IntakeConstants.INTAKE_COLLECTOR_MOTOR_CAN);

  private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(
    IntakeConstants.INTAKE_ENCODER_PIN
  );

  /**
   * Creates a new Intake PIDSubsystem.
   */
  public Intake() {
    super(
      // The PIDController used by the subsystem
      new PIDController(
        IntakeConstants.INTAKE_ARM_P,
        IntakeConstants.INTAKE_ARM_I,
        IntakeConstants.INTAKE_ARM_D
      )
    );

    getController().setTolerance(IntakeConstants.INTAKE_ARM_TARGET_TOLERANCE);
    armUp();
    enable();

    m_armEncoder.setDistancePerRotation(360);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_moverMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public double getMeasurement() {
    final double measurement = m_armEncoder.getDistance();

    SmartDashboard.putNumber("Intake Arm Degrees", measurement);

    return measurement;
  }

  /**
   * Sets the setpoint that tells the subsystem to move the intake arm
   * into the "up" position.
   */
  public void armUp() {
    setSetpoint(IntakeConstants.INTAKE_ARM_UP_DEGREE_POS);
  }

  /**
   * Sets the setpoint that tells the subsystem to move the intake arm
   * into the "down" position.
   */
  public void armDown() {
    setSetpoint(IntakeConstants.INTAKE_ARM_DOWN_DEGREE_POS);
  }

  /**
   * Turns on the collector motor.
   */
  public void startCollection() {
    m_collectorMotor.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_COLLECTOR_MOTOR_SPEED);
  }

  /**
   * Turns off the collector motor.
   */
  public void stopCollection() {
    m_collectorMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Lifts collector arm and runs collector in reverse.
   */
  public void vomit() {
    armUp();
    m_collectorMotor.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_COLLECTOR_MOTOR_SPEED);
  }
}
