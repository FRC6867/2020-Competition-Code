/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class Vomit extends CommandBase {
  private final Intake m_intake;
  private final Indexer m_indexer;
  private final Shooter m_shooter;

  /**
   * Creates a new Vomit command. This command runs all the motors in
   * reverse to empty balls and clear up jams. Will not end on it's own
   * - must be interrupted. This vomits all the subsystems at once.
   */
  public Vomit(Intake intake, Indexer indexer, Shooter shooter) {
    m_intake = intake;
    m_indexer = indexer;
    m_shooter = shooter;

    if (m_intake != null) {
      addRequirements(m_intake);
    }
    if (m_indexer != null) {
      addRequirements(m_indexer);
    }
    if (m_shooter != null) {
      addRequirements(m_shooter);
    }
  }

  /**
   * Creates a new Vomit command. This command runs all the motors in
   * reverse to empty balls and clear up jams. Will not end on it's own
   * - must be interrupted. This vomits only the intake.
   */
  public Vomit(Intake intake) {
    this(intake, null, null);
  }

  /**
   * Creates a new Vomit command. This command runs all the motors in
   * reverse to empty balls and clear up jams. Will not end on it's own
   * - must be interrupted. This vomits only the indedxer.
   */
  public Vomit(Indexer indexer) {
    this(null, indexer, null);
  }

  /**
   * Creates a new Vomit command. This command runs all the motors in
   * reverse to empty balls and clear up jams. Will not end on it's own
   * - must be interrupted. This vomits only the shooter.
   */
  public Vomit(Shooter shooter) {
    this(null, null, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake != null) {
      m_intake.armUp();
      m_intake.vomit();
    }
    if (m_indexer != null) {
      m_indexer.vomit();
    }
    if (m_shooter != null) {
      m_shooter.vomit();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_intake != null) {
      m_intake.stopCollection();
    }
    if (m_indexer != null) {
      m_indexer.stopIndexer();
    }
    if (m_shooter != null) {
      m_shooter.stopFeeder();
    }
  }

  // Don't end until interrupted.
  @Override
  public boolean isFinished() {
    return false;
  }
}
