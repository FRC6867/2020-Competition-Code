/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Vomittable;

public class Vomit extends CommandBase {
  private final Vomittable[] m_subsystems;

  /**
   * Creates a new Vomit commands that runs the {@link Vomittable#vomit()} method of the supplied subsystems.
   * Calls the {@link Vomittable#stopVomit()} when it ends.
   * Does not end on it's own.
   * 
   * @param subsystems A list of subsystems that implement the {@link Vomittable} interface
   */
  public Vomit(Vomittable... subsystems) {
    m_subsystems = subsystems;

    addRequirements(m_subsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (Vomittable subsystem : m_subsystems) {
      subsystem.vomit();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (Vomittable subsystem : m_subsystems) {
      subsystem.stopVomit();
    }
  }

  // Don't end until interrupted.
  @Override
  public boolean isFinished() {
    return false;
  }
}
