/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.IndexerConstants;

/**
 * Subsystem that handles indexing of the balls
 * and the operation of the big bown in the front
 * of the robot.
 */
public class Indexer extends SubsystemBase {
  private final VictorSPX m_rightIndexerMotor = new VictorSPX(IndexerConstants.RIGHT_MOTOR_CAN);
  private final VictorSPX m_leftIndexerMotor = new VictorSPX(IndexerConstants.LEFT_MOTOR_CAN);

  private final ColorSensorV3 m_ballSensor = new ColorSensorV3(I2C.Port.kOnboard);

  /**
   * Creates a new Indexer subsystem using {@link IndexerConstants}.
   */
  public Indexer() {
    // Motor config
    m_rightIndexerMotor.configFactoryDefault();
    m_leftIndexerMotor.configFactoryDefault();

    m_rightIndexerMotor.setInverted(IndexerConstants.RIGHT_MOTOR_INVERTED);
    m_leftIndexerMotor.setInverted(IndexerConstants.LEFT_MOTOR_INVERTED);
  }

  /**
   * Runs both sides of the Indexer at max
   * throttled speed.
   * 
   * @see {@link Indexer#run()}
   */
  public void runAll() {
    run(1, 1);
  }

  /**
   * Stops both sides of the indexer.
   */
  public void stop() {
    run(0, 0);
  }

  /**
   * Runs both sides of the indexer. Speed
   * is multiplied by throttle.
   * 
   * @param rightSpeed Right side speed
   * @param leftSpeed Left side speed
   */
  public void run(double rightSpeed, double leftSpeed) {
    runRight(rightSpeed);
    runLeft(leftSpeed);
  }

  /**
   * Runs the right side of the indexer. Speed
   * is multiplied by throttle.
   * 
   * @param speed The speed to run the motor at
   */
  public void runRight(double speed) {
    setMotor(m_rightIndexerMotor, speed);
  }

  /**
   * Runs the left side of the indexer. Speed
   * is multiplied by throttle.
   * 
   * @param speed The speed to run the motor at
   */
  public void runLeft(double speed) {
    setMotor(m_leftIndexerMotor, speed);
  }

  // Handles throttle
  private void setMotor(VictorSPX motor, double speed) {
    motor.set(ControlMode.PercentOutput, IndexerConstants.INDEXER_SPEED * speed);
  }

  /**
   * @return Whether there is a ball ready to be fed
   */
  public boolean ballReady() {
    return m_ballSensor.getProximity() >= IndexerConstants.BALL_CLOSE_DISTANCE; // Alternate implementation, needs tuning.
  }


  @Override
  public void periodic() {
    // Log info
    SmartDashboard.putBoolean("Ball Ready", ballReady());
    SmartDashboard.putNumber("Ball Proximity", m_ballSensor.getProximity());
  }
}