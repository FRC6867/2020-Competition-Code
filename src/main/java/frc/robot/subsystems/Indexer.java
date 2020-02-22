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

public class Indexer extends SubsystemBase {
  private final VictorSPX m_indexerMotor1= new VictorSPX(IndexerConstants.INDEXER_MOTOR_1_CAN);
  private final VictorSPX m_indexerMotor2= new VictorSPX(IndexerConstants.INDEXER_MOTOR_2_CAN);

  private final ColorSensorV3 m_ballSensor = new ColorSensorV3(I2C.Port.kOnboard);

  /**
   * Creates a new Indexer subsystem.
   */
  public Indexer() {
    // Motor config
    m_indexerMotor1.configFactoryDefault();
    m_indexerMotor2.configFactoryDefault();

    m_indexerMotor1.setInverted(IndexerConstants.INDEXER_MOTOR_1_INVERTED);
    m_indexerMotor2.setInverted(IndexerConstants.INDEXER_MOTOR_2_INVERTED);
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

  // Runs both motors at their respective speeds
  private void setIndexerMotors(double speedMultiplier) {
    m_indexerMotor1.set(ControlMode.PercentOutput, speedMultiplier * IndexerConstants.INDEXER_MOTOR_1_SPEED);
    m_indexerMotor2.set(ControlMode.PercentOutput, speedMultiplier * IndexerConstants.INDEXER_MOTOR_2_SPEED);
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