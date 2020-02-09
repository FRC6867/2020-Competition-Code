/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  // Drives
  private final CANSparkMax m_frontLeftDrive = new CANSparkMax(DriveTrainConstants.BACK_LEFT_MOTOR_CAN, MotorType.kBrushless);
  private final CANSparkMax m_backLeftDrive = new CANSparkMax(DriveTrainConstants.FRONT_LEFT_MOTOR_CAN, MotorType.kBrushless);
  private final CANSparkMax m_frontRightDrive = new CANSparkMax(DriveTrainConstants.FRONT_RIGHT_MOTOR_CAN, MotorType.kBrushless);
  private final CANSparkMax m_backRightDrive = new CANSparkMax(DriveTrainConstants.BACK_RIGHT_MOTOR_CAN, MotorType.kBrushless);

  private final AHRS m_navX = new AHRS(Port.kMXP);

  private double m_speedThrottle = DriveTrainConstants.SPEED_THROTTLE;

  // Encoders
  private final Encoder m_rightEncoder = new Encoder(
    DriveTrainConstants.RIGHT_DRIVE_ENCODER_PINS[0],
    DriveTrainConstants.RIGHT_DRIVE_ENCODER_PINS[1],
    DriveTrainConstants.RIGHT_ENCODER_INVERTED,
    Encoder.EncodingType.k4X
  );
  private final Encoder m_leftEncoder = new Encoder(
    DriveTrainConstants.LEFT_DRIVE_ENCODER_PINS[0],
    DriveTrainConstants.LEFT_DRIVE_ENCODER_PINS[1],
    DriveTrainConstants.LEFT_ENCODER_INVERTED,
    Encoder.EncodingType.k4X
  );

  // Ultrasonic distance sensor
  private final Ultrasonic m_distanceSensor = new Ultrasonic(
    DriveTrainConstants.ULTRASONIC_TRIGGER_PIN,
    DriveTrainConstants.ULTRASONIC_ECHO_PIN
  );


  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    // Motor directions and following
    m_frontRightDrive.setInverted(DriveTrainConstants.RIGHT_MOTOR_INVERTED);
    m_backRightDrive.follow(m_frontRightDrive);
    m_frontLeftDrive.setInverted(DriveTrainConstants.LEFT_MOTOR_INVERTED);
    m_backLeftDrive.follow(m_frontLeftDrive);

    // Set encoder distance values
    m_rightEncoder.setDistancePerPulse(DriveTrainConstants.RIGHT_ENCODER_TICK_DISTANCE);
    m_leftEncoder.setDistancePerPulse(DriveTrainConstants.LEFT_ENCODER_TICK_DISTANCE);

    // Start distance sensor
    m_distanceSensor.setAutomaticMode(true);

    reset(); // Make sure we start off fresh
  }



  /**
   * Attempts to go straight, correcting itself using gyro.
   * 
   * @Warning !! Remember to call DriveTrain.reset() before using !!
   * 
   * @param speed - Goes straighter the faster it is.
   */  
  public void driveStraight(double speed) {
    // Calculates error based on gyro heading
    final double error = -getHeading();
    final double turnMod = error * DriveTrainConstants.TURN_CORRECTION_P;

    drive(speed - turnMod, speed + turnMod);
  }

  /**
   * Sets both motors. Throttle is handled.
   * 
   * @param rightSpeed Speed of the right motor
   * @param leftSpeed Speed of the left motor
   */
  public void drive(double rightSpeed, double leftSpeed) {
    driveRight(rightSpeed);
    driveLeft(leftSpeed);
  }

  public void driveRight(double speed) {
    setMotor(m_frontRightDrive, speed);
  }

  public void driveLeft(double speed) {
    setMotor(m_frontLeftDrive, speed);
  }

  private void setMotor(CANSparkMax motor, double rawSpeed) {
    motor.set(rawSpeed * DriveTrainConstants.SPEED_THROTTLE);
  }

  public void setFineControl(boolean fineControl) {
    if (fineControl) {
      m_speedThrottle = m_speedThrottle / 2;
    } else {
      m_speedThrottle = m_speedThrottle * 2;
    }
    updateSpeedThrottle();
  }

  /**
   * Resets the values of the {@link DriveTrain}'s sensors.
   */
  public void reset() {
    m_navX.reset();

    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * @return Average distance driven, in inches
   */
  public double getDistanceDriven() { // Average distance
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
  }

  /**
   * @return Left encoder distance, in inches
   */
  public double getLeftEncoder() {
    return m_leftEncoder.getDistance();
  }

  /**
   * @return Right encoder distance, in inches
   */
  public double getRightEncoder() {
    return m_rightEncoder.getDistance();
  }
  
  /**
   * @return Relative gyro heading, in degrees
   */
  public double getHeading() {
    return m_navX.getAngle();
  }

  /**
   * @return Distance to nearest object (front), in inches
   */
  public double getDistanceToObject() {
    // m_distanceSensor.ping(); // If we would like to turn off automatic mode
    return m_distanceSensor.getRangeInches();
  }


  private void updateSpeedThrottle() {
    SmartDashboard.putNumber("DriveTrain Throttle", m_speedThrottle);
  }

  @Override
  public void periodic() {
    m_speedThrottle = SmartDashboard.getNumber("DriveTrain Throttle", m_speedThrottle);
    updateSpeedThrottle();

    // Log data
    SmartDashboard.putNumber("Distance Driven", getDistanceDriven());
    SmartDashboard.putNumber("Distance From Object",getDistanceToObject());
    SmartDashboard.putNumber("Heading", getHeading());
  }
}