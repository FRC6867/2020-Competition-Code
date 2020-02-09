/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private final VictorSPX m_indexerMotor = new VictorSPX(IndexerConstants.INDEXER_MOTOR_CAN);

  private final ColorSensorV3 m_ballSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color BALL_COLOR = ColorMatch.makeColor(
    IndexerConstants.BALL_COLOR_R,
    IndexerConstants.BALL_COLOR_G, 
    IndexerConstants.BALL_COLOR_B
  );

  /**
   * Creates a new Indexer subsystem.
   */
  public Indexer() {
    // Sets motor
    m_indexerMotor.setInverted(IndexerConstants.INDEXER_MOTOR_INVERTED);

    // Inits ball sensor
    m_colorMatcher.addColorMatch(BALL_COLOR);
  }

  /**
   * Turns on the indexer motor at {@link IndexerConstants#m_indexerMotor_SPEED} speed.
   */
  public void startIndexer() {
    m_indexerMotor.set(ControlMode.PercentOutput, IndexerConstants.INDEXER_MOTOR_SPEED);
  }

  /**
   * Turns off the indexer motor
   */
  public void stopIndexer() {
    m_indexerMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Runs the motor in reverse.
   */
  public void vomit() {
    m_indexerMotor.set(ControlMode.PercentOutput, -IndexerConstants.INDEXER_MOTOR_SPEED);
  }

  /**
   * 
   * @return Whether there is a ball ready to be fed
   */
  public boolean ballReady() {
    // return m_ballSensor.getProximity() >= IndexerConstants.BALL_CLOSE_DISTANCE; // Alternate implementation, needs tuning.

    Color rawColor = m_ballSensor.getColor();
    ColorMatchResult colorMatch = m_colorMatcher.matchClosestColor(rawColor);

    SmartDashboard.putNumber("Red", rawColor.red); // Temporary for tuning
    SmartDashboard.putNumber("Green", rawColor.green);
    SmartDashboard.putNumber("Blue", rawColor.blue);

    return (colorMatch.color == BALL_COLOR);
  }


  @Override
  public void periodic() {
    // Log info
    SmartDashboard.putBoolean("Ball Ready", ballReady());
    SmartDashboard.putNumber("Ball Proximity", m_ballSensor.getProximity());
  }
}
