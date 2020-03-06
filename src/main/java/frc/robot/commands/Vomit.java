/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;

/**
 * A {@link Shooter}-based command that runs the Shooter
 * at low speed to dump balls out instead of shooting them.
 */
public class Vomit extends CommandBase {
  private final Shooter m_shooter;

  /**
   * Creates a new Vomit command.
   * 
   * @param shooter The Shooter subsystem
   */
  public Vomit(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.vomit();
  }

  @Override
  public void execute() {
    if (m_shooter.isReady()) {
      m_shooter.runFeeder();
    } else {
      m_shooter.stopFeeder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopVomit();
  }

  // Don't end until interrupted.
  @Override
  public boolean isFinished() {
    return false;
  }
}
