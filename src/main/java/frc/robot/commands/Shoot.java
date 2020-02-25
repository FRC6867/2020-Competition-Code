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

/**
 * A fairly simple command. I decided to write this in it's
 * own class rather than inline because ¯\_(ツ)_/¯ I'm lazy
 */
public class Shoot extends CommandBase {
  private final Shooter m_shooter;
  private final Indexer m_indexer;

  /**
   * Creates a new Shoot command. Spins up the shooter on
   * start, runs feeder when shooter and indexer are ready.
   * Never ends, must be interrupted, at which point it stops
   * the shooter.
   * 
   * @param shooter The {@link Shooter} subsystem
   * @param indexer the {@link Indexer} subsystem
   */
  public Shoot(Shooter shooter, Indexer indexer) {
    m_shooter = shooter;
    m_indexer = indexer;

    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.shooterReady()) {// && m_indexer.ballReady()) {
      m_shooter.runFeeder();
    } else {
      m_shooter.stopFeeder();
    }
    m_shooter.runFeeder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
