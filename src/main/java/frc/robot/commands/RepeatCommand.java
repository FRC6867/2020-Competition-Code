/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command which runs another command several times in a row.
 */
public class RepeatCommand extends CommandBase {
  private int m_currentReps = 0;
  private int m_targetReps;

  private Command m_command;

  /**
   * Creates a new RepeatCommand, which runs the commandToRun runTimes times.
   * Does not need any requirements.
   */
  public RepeatCommand(Command commandToRun, int runTimes) {
    m_command = commandToRun;
    m_targetReps = runTimes;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_command.isScheduled()) {
      m_command.schedule();
      m_currentReps++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_command.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_currentReps >= m_targetReps) && m_command.isFinished();
  }
}
