/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;

import frc.robot.subsystems.Indexer;

/**
 * Command that controls the manual functions of the {@link Indexer}
 * subsystem.
 */
public class IndexerControl extends CommandBase {
  private final Indexer m_indexer;

  private final Button m_rightForwardButton;
  private final Button m_rightBackwardButton;
  private final Button m_leftForwardButton;
  private final Button m_leftBackwardButton;

  /**
   * Creates a new IndexerControl command.
   * 
   * @param indexer The Indexer subsystem
   */
  public IndexerControl(Button rightForwardButton,
                        Button rightBackwardButton,
                        Button leftForwardButton,
                        Button leftBackwardButton,
                        Indexer indexer) {

    m_rightForwardButton = rightForwardButton;
    m_rightBackwardButton = rightBackwardButton;
    m_leftForwardButton = leftForwardButton;
    m_leftBackwardButton = leftBackwardButton;

    m_indexer = indexer;
    
    addRequirements(m_indexer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Right
    m_indexer.runRight(
      getNumFromButton(m_rightForwardButton) - getNumFromButton(m_rightBackwardButton)
    );

    // Left
    m_indexer.runLeft(
      getNumFromButton(m_leftForwardButton) - getNumFromButton(m_leftBackwardButton)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private int getNumFromButton(Button button) {
    return button.get()? 1:0;
  }
}
