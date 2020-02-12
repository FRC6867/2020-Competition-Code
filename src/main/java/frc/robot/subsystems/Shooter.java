/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends PIDSubsystem {
  // Motors
  private final VictorSPX m_shooterMotor1 = new VictorSPX(ShooterConstants.SHOOTER_MOTOR_1_CAN);
  private final VictorSPX m_shooterMotor2 = new VictorSPX(ShooterConstants.SHOOTER_MOTOR_2_CAN);
  private final VictorSPX m_feederMotor = new VictorSPX(ShooterConstants.FEEDER_MOTOR_CAN);

  // Encoders
  private final Encoder m_shooterEncoder = new Encoder(
    ShooterConstants.SHOOTER_ENCODER_PINS[0],
    ShooterConstants.SHOOTER_ENCODER_PINS[1],
    ShooterConstants.SHOOTER_ENCODER_INVERTED,
    Encoder.EncodingType.k4X
  );

  /**
   * Creates a new Shooter PIDSubsystem.
   */
  public Shooter() {
    super(
      // The PIDController used by the subsystem
      new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D)
    );

    // Controller config
    getController().setTolerance(ShooterConstants.SHOOTER_TARGET_RPM_TOLERANCE);
    setSetpoint(ShooterConstants.SHOOTER_TARGET_RPM);
    setPID();

    // Encoder config
    // One full rotation will be one unit
    m_shooterEncoder.setDistancePerPulse(1 / ShooterConstants.SHOOTER_ENCODER_TICKS_PER_ROTATION);

    // Motor config
    m_shooterMotor1.setInverted(ShooterConstants.SHOOTER_MOTORS_INVERTED);
    m_shooterMotor2.setInverted(ShooterConstants.SHOOTER_MOTORS_INVERTED);
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
    m_shooterMotor1.set(ControlMode.PercentOutput, output);
    m_shooterMotor2.set(ControlMode.PercentOutput, -output); // These two work together

    // Log
    double speed = getMeasurement();
    SmartDashboard.putNumber("Shooter RPM", speed);
    SmartDashboard.putNumber("Shooter RPM Graph", speed);
  }

  @Override
  public double getMeasurement() {
    return m_shooterEncoder.getRate() * 60;
  }

  /**
   * @return Whether the shooter is up-to-speed and ready
   */
  public boolean shooterReady() {
    return getController().atSetpoint();
  }


  /**
   * Runs the feeder motor(s) at {@link ShooterConstants#FEEDER_SPEED} speed.
   */
  public void runFeeder() {
    m_feederMotor.set(ControlMode.PercentOutput, ShooterConstants.FEEDER_SPEED);
  }

  /**
   * Stops the feeder motor(s).
   */
  public void stopFeeder() {
    m_feederMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Runs the feeder motor(s) in reverse. Flywheel cannot reverse.
   */
  public void vomit() {
    m_feederMotor.set(ControlMode.PercentOutput, -ShooterConstants.FEEDER_SPEED);
  }
}
