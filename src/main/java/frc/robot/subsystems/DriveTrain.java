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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.ComponentIDConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.PIDConstants;


public class DriveTrain extends SubsystemBase {
  // Drives
  private final VictorSPX FRONT_LEFT_DRIVE = new VictorSPX(ComponentIDConstants.FRONT_LEFT_DRIVE_CAN);
  private final VictorSPX BACK_LEFT_DRIVE = new VictorSPX(ComponentIDConstants.BACK_LEFT_DRIVE_CAN);
  private final VictorSPX FRONT_RIGHT_DRIVE = new VictorSPX(ComponentIDConstants.FRONT_RIGHT_DRIVE_CAN);
  private final VictorSPX BACK_RIGHT_DRIVE = new VictorSPX(ComponentIDConstants.BACK_RIGHT_DRIVE_CAN);

  private final AHRS NAVX = new AHRS(Port.kMXP);

  // Encoders
  private final Encoder RIGHT_ENCODER = new Encoder(
    ComponentIDConstants.RIGHT_DRIVE_ENCODER_PIN_1,
    ComponentIDConstants.RIGHT_DRIVE_ENCODER_PIN_2,
    false,
    Encoder.EncodingType.k4X
  );
  private final Encoder LEFT_ENCODER = new Encoder(
    ComponentIDConstants.LEFT_DRIVE_ENCODER_PIN_1,
    ComponentIDConstants.LEFT_DRIVE_ENCODER_PIN_2,
    true, // Left encoder is inverted
    Encoder.EncodingType.k4X
  );

  // Ultrasonic distance sensor
  private final Ultrasonic DISTANCE_SENSOR = new Ultrasonic(
    ComponentIDConstants.ULTRASONIC_TRIGGER_PIN,
    ComponentIDConstants.ULTRASONIC_ECHO_PIN
  );


  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    // Motor directions
    FRONT_RIGHT_DRIVE.setInverted(true);
    BACK_RIGHT_DRIVE.setInverted(true);
    FRONT_LEFT_DRIVE.setInverted(false);
    BACK_LEFT_DRIVE.setInverted(false);

    // Set encoder distance values
    RIGHT_ENCODER.setDistancePerPulse(0.01136);
    LEFT_ENCODER.setDistancePerPulse(0.01134);

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
    final double turnMod = error * PIDConstants.TURN_CORRECTION_P;

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
    setMotor(BACK_RIGHT_DRIVE, speed);
  }

  public void driveLeft(double speed) {
    setMotor(FRONT_LEFT_DRIVE, speed);
    setMotor(BACK_LEFT_DRIVE, speed);
  }

  private void setMotor(VictorSPX motor, double rawSpeed) {
    motor.set(ControlMode.PercentOutput, rawSpeed * SpeedConstants.MASTER_THROTTLE);
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