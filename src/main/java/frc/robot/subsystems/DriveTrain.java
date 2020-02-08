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
  private final CANSparkMax FRONT_LEFT_DRIVE = new CANSparkMax(DriveTrainConstants.BACK_LEFT_MOTOR_CAN, MotorType.kBrushless);
  private final CANSparkMax BACK_LEFT_DRIVE = new CANSparkMax(DriveTrainConstants.FRONT_LEFT_MOTOR_CAN, MotorType.kBrushless);
  private final CANSparkMax FRONT_RIGHT_DRIVE = new CANSparkMax(DriveTrainConstants.FRONT_RIGHT_MOTOR_CAN, MotorType.kBrushless);
  private final CANSparkMax BACK_RIGHT_DRIVE = new CANSparkMax(DriveTrainConstants.BACK_RIGHT_MOTOR_CAN, MotorType.kBrushless);

  private final AHRS NAVX = new AHRS(Port.kMXP);

  // Encoders
  private final Encoder RIGHT_ENCODER = new Encoder(
    DriveTrainConstants.RIGHT_DRIVE_ENCODER_PINS[0],
    DriveTrainConstants.RIGHT_DRIVE_ENCODER_PINS[1],
    DriveTrainConstants.RIGHT_ENCODER_INVERTED,
    Encoder.EncodingType.k4X
  );
  private final Encoder LEFT_ENCODER = new Encoder(
    DriveTrainConstants.LEFT_DRIVE_ENCODER_PINS[0],
    DriveTrainConstants.LEFT_DRIVE_ENCODER_PINS[1],
    DriveTrainConstants.LEFT_ENCODER_INVERTED, // Left encoder is inverted
    Encoder.EncodingType.k4X
  );

  // Ultrasonic distance sensor
  private final Ultrasonic DISTANCE_SENSOR = new Ultrasonic(
    DriveTrainConstants.ULTRASONIC_TRIGGER_PIN,
    DriveTrainConstants.ULTRASONIC_ECHO_PIN
  );


  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    // Motor directions and following
    FRONT_RIGHT_DRIVE.setInverted(true);
    BACK_RIGHT_DRIVE.follow(FRONT_RIGHT_DRIVE);
    FRONT_LEFT_DRIVE.setInverted(false);
    BACK_LEFT_DRIVE.follow(FRONT_LEFT_DRIVE);

    // Set encoder distance values
    // TODO: Get encoder distances
    RIGHT_ENCODER.setDistancePerPulse(1);
    LEFT_ENCODER.setDistancePerPulse(1);

    // Start distance sensor
    DISTANCE_SENSOR.setAutomaticMode(true);

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
    setMotor(FRONT_RIGHT_DRIVE, speed);
  }

  public void driveLeft(double speed) {
    setMotor(FRONT_LEFT_DRIVE, speed);
  }

  private void setMotor(CANSparkMax motor, double rawSpeed) {
    motor.set(rawSpeed * DriveTrainConstants.SPEED_THROTTLE);
  }


  /**
   * Resets the values of the {@link DriveTrain}'s sensors.
   */
  public void reset() {
    NAVX.reset();

    LEFT_ENCODER.reset();
    RIGHT_ENCODER.reset();
  }

  /**
   * @return Average encoder distance, in inches
   */
  public double getDistanceDriven() { // Average distance
    return (LEFT_ENCODER.getDistance() + RIGHT_ENCODER.getDistance()) / 2;
  }

  public double getLeftEncoder() {
    return LEFT_ENCODER.getDistance();
  }

  public double getRightEncoder() {
    return RIGHT_ENCODER.getDistance();
  }
  
  /**
   * @return Relative gyro heading, in degrees
   */
  public double getHeading() {
    return NAVX.getAngle();
  }

  /**
   * @return Distance to nearest object (front), in inches
   */
  public double getDistanceToObject() {
    // DISTANCE_SENSOR.ping(); // If we would like to turn off automatic mode
    return DISTANCE_SENSOR.getRangeInches();
  }


  @Override
  public void periodic() {
    // Log data
    SmartDashboard.putNumber("Distance Driven", getDistanceDriven());
    SmartDashboard.putNumber("Distance From Object",getDistanceToObject());
    SmartDashboard.putNumber("Heading", getHeading());
  }
}