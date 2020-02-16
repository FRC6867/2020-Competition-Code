/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

public class Vomit extends CommandBase {
  private final Shooter m_shooter;
  private final Indexer m_indexer;

  /**
   * Creates a new Vomit command. This command runs all the motors in
   * reverse to empty balls and clear up jams. Will not end on it's own
   * - must be interrupted.
   * 
   * @param shooter The {@link Shooter} subsystem
   * @param indexer The {@link Indexer} subsystem
   */
  public Vomit(Shooter shooter, Indexer indexer) {
    // TODO: Add vomit functionality to other subsystems
    m_shooter = shooter;
    m_indexer = indexer;

    addRequirements(m_shooter, m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.vomit();
    m_shooter.vomit();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_shooter.stopFeeder();
  }

  // Don't end until interrupted.
  @Override
  public boolean isFinished() {
    return false;
  }
}
