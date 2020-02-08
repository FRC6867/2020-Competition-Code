/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbArm;

public class Climb extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbArm m_climbArm;
  private final double m_speed;

  /**
   * Creates a new Climb.
   * 
   * @param speed Basically the direction to move
   * @param climbArm ClimbArm subsystem
   */
  public Climb(double speed, ClimbArm climbArm) {
    m_climbArm = climbArm;
    m_speed = speed;
    
    addRequirements(climbArm);
  }

  // Do stuff
  @Override
  public void execute() {
    System.out.println(m_speed);
    m_climbArm.climb(m_speed);
  }

  // Default command - never stop
  @Override
  public boolean isFinished() {
    return false;
  }

  // Stop motor when button is no longer held
  @Override
  public void end(boolean interrupted) {
    m_climbArm.climb(0);
  }
}