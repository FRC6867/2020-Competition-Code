/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends PIDSubsystem {
  // Motors
  private final TalonSRX m_shooterMotor1 = new TalonSRX(ShooterConstants.SHOOTER_MOTOR_1_CAN);
  private final TalonSRX m_shooterMotor2 = new TalonSRX(ShooterConstants.SHOOTER_MOTOR_2_CAN);
  private final VictorSPX m_feederMotor = new VictorSPX(ShooterConstants.FEEDER_MOTOR_CAN);

  // Encoders
  private final Encoder m_shooterEncoder = new Encoder(
    ShooterConstants.SHOOTER_ENCODER_PINS[0],
    ShooterConstants.SHOOTER_ENCODER_PINS[1],
    ShooterConstants.SHOOTER_ENCODER_INVERTED
  );

  /**
   * Creates a new Shooter PIDSubsystem.
   */
  public Shooter() {
    // PID config
    super(
      // The PIDController used by the subsystem
      new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D)
    );
    getController().setTolerance(ShooterConstants.SHOOTER_TARGET_RPM_TOLERANCE);
    setSetpoint(ShooterConstants.SHOOTER_TARGET_RPM);
    SmartDashboard.putNumber("Shooter F", ShooterConstants.SHOOTER_F);

    // Encoder config
    m_shooterEncoder.reset();
    m_shooterEncoder.setDistancePerPulse(1.0 / (double) ShooterConstants.SHOOTER_ENCODER_CPR);

    // Motor config
    m_shooterMotor1.configFactoryDefault();
    m_shooterMotor2.configFactoryDefault();
    m_feederMotor.configFactoryDefault();

    m_shooterMotor1.setInverted(ShooterConstants.SHOOTER_MOTORS_INVERTED);
    m_shooterMotor2.setInverted(ShooterConstants.SHOOTER_MOTORS_INVERTED);
    m_shooterMotor2.follow(m_shooterMotor1); // Second motor should mirror first one
    m_feederMotor.setInverted(ShooterConstants.FEEDER_MOTOR_INVERTED);

    m_shooterMotor1.setNeutralMode(NeutralMode.Coast); // Make sure shooter doesn't brake.
    m_shooterMotor2.setNeutralMode(NeutralMode.Coast);
  }

  private void setPID() {
    getController().setPID(
      SmartDashboard.getNumber("Shooter P", getController().getP()),
      SmartDashboard.getNumber("Shooter I", getController().getI()),
      SmartDashboard.getNumber("Shooter D", getController().getD())
    );

    SmartDashboard.putNumber("Shooter P", getController().getP());
    SmartDashboard.putNumber("Shooter I", getController().getI());
    SmartDashboard.putNumber("Shooter D", getController().getD());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor1.set(ControlMode.PercentOutput, output + getFeedForward(setpoint)); // Other motor will follow

    SmartDashboard.putNumber("Shooter Output", output);
  }

  @Override
  public double getMeasurement() {
    return m_shooterEncoder.getRate() * 60; // .getRate() returns units per sec, we need per min.
  }

  /**
   * Returns the basic kF * setpoint feedforward
   */
  private double getFeedForward(double setpoint) {
    return ShooterConstants.SHOOTER_F * setpoint;
  }

  /**
   * @return Whether the shooter is up-to-speed and ready
   */
  public boolean isReady() {
    final boolean atSetpoint = getController().atSetpoint();

    SmartDashboard.putBoolean("Shooter Ready", atSetpoint);

    return atSetpoint;
  }

  /**
   * Log speed
   * @param speed Current speed
   */
  private void updateData(double speed) {
    SmartDashboard.putNumber("Shooter RPM", speed);
  }


  /**
   * Runs the feeder motor at {@link ShooterConstants#FEEDER_SPEED} speed.
   */
  public void runFeeder() {
    //System.out.println("feed");
    m_feederMotor.set(ControlMode.PercentOutput, ShooterConstants.FEEDER_SPEED);
  }

  /**
   * Stops the feeder motor.
   */
  public void stopFeeder() {
    //System.out.println("no feed");
    m_feederMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Runs the feeder motor in reverse. Flywheel cannot reverse.
   */
  public void vomit() {
    setSetpoint(ShooterConstants.SHOOTER_VOMIT_TARGET_RPM);
    enable();
  }

  /**
   * Stops vomitting.
   */
  public void stopVomit() {
    disable();
    setSetpoint(ShooterConstants.SHOOTER_TARGET_RPM);
  }

  // Call parent periodic + log data
  @Override
  public void periodic() {
    setPID();
    super.periodic();
    updateData(getMeasurement());
  }
}
