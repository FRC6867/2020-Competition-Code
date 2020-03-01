/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.Vomittable;

import edu.wpi.first.wpilibj.DutyCycleEncoder; // In case we go back to absolute
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants.IntakeConstants;

public class Intake extends PIDSubsystem implements Vomittable {
  private final TalonSRX m_moverMotor = new TalonSRX(IntakeConstants.INTAKE_MOVER_MOTOR_CAN);
  private final TalonSRX m_collectorMotor = new TalonSRX(IntakeConstants.INTAKE_COLLECTOR_MOTOR_CAN);

  // private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder( // In case we go back to absolute
  //   IntakeConstants.INTAKE_ENCODER_PIN
  // );
  private final Encoder m_armEncoder = new Encoder(
    IntakeConstants.INTAKE_ENCODER_PINS[0],
    IntakeConstants.INTAKE_ENCODER_PINS[1],
    IntakeConstants.INTAKE_ENCODER_INVERTED
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

    // Motor config
    m_moverMotor.configFactoryDefault();
    m_collectorMotor.configFactoryDefault();

    m_moverMotor.setInverted(IntakeConstants.INTAKE_MOVER_MOTOR_INVERTED);
    m_collectorMotor.setInverted(IntakeConstants.INTAKE_COLLECTOR_MOTOR_INVERTED);

    m_moverMotor.setNeutralMode(NeutralMode.Brake); // Doesn't let arm fall
    m_collectorMotor.setNeutralMode(NeutralMode.Coast); // So collector keeps spinning

    // Config amp limits
    m_collectorMotor.configPeakCurrentLimit(55); // Collector was browning out or something
    
    // Encoder setup
    m_armEncoder.reset(); // For relative
    //m_armEncoder.setDistancePerRotation(360); // For absolute

    // PID setup
    getController().setTolerance(IntakeConstants.INTAKE_ARM_TARGET_TOLERANCE);
    setSetpoint(IntakeConstants.INTAKE_ARM_DEGREE_DOWN_POS);
    //enable(); // Disabled until we have proper PID
  }

  private void setPID() {
    getController().setPID(
      SmartDashboard.getNumber("Intake Arm P", getController().getP()),
      SmartDashboard.getNumber("Intake Arm I", getController().getI()),
      SmartDashboard.getNumber("Intake Arm D", getController().getD())
    );

    SmartDashboard.putNumber("Intake Arm P", getController().getP());
    SmartDashboard.putNumber("Intake Arm I", getController().getI());
    SmartDashboard.putNumber("Intake Arm D", getController().getD());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    SmartDashboard.putNumber("Arm Output", output);
    m_moverMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public double getMeasurement() {
    final double rawMeasurement = m_armEncoder.getDistance();
    final double measurement = rawMeasurement / 1000 * 90;

    SmartDashboard.putNumber("Intake Arm Degrees", measurement);

    return measurement;
  }

  // Temporary
  public void armUpManual() {
    m_moverMotor.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_ARM_MANUAL_SPEED);
  }

  // Temporary
  public void armDownManual() {
    m_moverMotor.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_ARM_MANUAL_SPEED);
  }

  // Temporary
  public void stopArmManual() {
    m_moverMotor.set(ControlMode.PercentOutput, 0);
  }

  // Undefined
  public void armUp() {
  }

  // Undefined
  public void armDown() {
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

  /**
   * Stops vomitting
   */
  public void stopVomit() {
    stopCollection();
  }

  // TODO: Disabled until proper PID control
  // // Update PID all the time for tuning
  // @Override
  // public void periodic() {
  //   setPID();
  //   super.periodic();
  // }
}
