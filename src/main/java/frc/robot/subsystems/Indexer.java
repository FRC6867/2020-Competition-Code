/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vomittable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase implements Vomittable {
  private final VictorSPX m_mainIndexerMotor = new VictorSPX(IndexerConstants.INDEXER_MAIN_MOTOR_CAN);
  private final VictorSPX m_transferMotor = new VictorSPX(IndexerConstants.TRANSFER_MOTOR_CAN);

  private final ColorSensorV3 m_ballSensor = new ColorSensorV3(I2C.Port.kOnboard);

  /**
   * Creates a new Indexer subsystem.
   */
  public Indexer() {
    // Motor config
    m_mainIndexerMotor.configFactoryDefault();
    m_transferMotor.configFactoryDefault();

    m_mainIndexerMotor.setInverted(IndexerConstants.INDEXER_MAIN_MOTOR_INVERTED);
    m_transferMotor.setInverted(IndexerConstants.TRANSFER_MOTOR_INVERTED);
  }

  /**
   * Turns on the indexer motor at {@link IndexerConstants#m_indexerMotor_SPEED} speed.
   */
  public void startIndexer() {
    setIndexerMotors(1);
  }

  /**
   * Turns off the indexer motor
   */
  public void stopIndexer() {
    setIndexerMotors(0);
  }

  /**
   * Runs the motor in reverse.
   */
  public void vomit() {
    setIndexerMotors(-1);
  }

  /**
   * Stops vomitting.
   */
  public void stopVomit() {
    stopIndexer();
  }

  // Runs both motors at their respective speeds
  private void setIndexerMotors(double speedMultiplier) {
    m_mainIndexerMotor.set(ControlMode.PercentOutput, speedMultiplier * IndexerConstants.INDEXER_MAIN_MOTOR_SPEED);
    runTransfer(speedMultiplier);
  }

  public void runTransfer(double speedMultiplier) {
    m_transferMotor.set(ControlMode.PercentOutput, speedMultiplier * IndexerConstants.TRANSFER_MOTOR_SPEED);
  }

  public void stopTransfer() {
    runTransfer(0);
  }

  /**
   * 
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