/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Manages intaking balls from the ground
 */
public class FloorIntake extends CommandBase {
  private final Intake m_intake;
  private final Indexer m_indexer;

  /**
   * Creates a new FloorIntake command, which lowers the intake arm,
   * turns on collector motor and transfer motor.
   */
  public FloorIntake(Intake intake, Indexer indexer) {
    m_intake = intake;
    m_indexer = indexer;

    addRequirements(m_intake, m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.armDown();
    m_intake.startCollection();
    m_indexer.runTransfer(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.armUp();
    m_intake.stopCollection();
    m_indexer.stopTransfer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
