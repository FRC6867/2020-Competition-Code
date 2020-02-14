/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.VideoCamera;

/**
 * yay i got a camera...
 * 
 * Just don't ask me how it works ok
 */
public class Camera extends SubsystemBase {
  private VideoCamera m_camera;

  /**
   * Creates a new Camera thing.
   */
  public Camera() {
    m_camera = CameraServer.getInstance().startAutomaticCapture("Camera", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
