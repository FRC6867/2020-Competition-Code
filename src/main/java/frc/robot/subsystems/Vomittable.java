/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link Subsystem} that can vomit - reverse its motors.
 * Useful for clearing out jams and such.
 */
public interface Vomittable extends Subsystem {
    public abstract void vomit();
    public abstract void stopVomit();
}
