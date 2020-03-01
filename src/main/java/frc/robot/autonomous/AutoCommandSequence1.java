/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.commands.TurnDegrees;
import frc.robot.commands.DriveForward;
import frc.robot.commands.FloorIntake;
import frc.robot.commands.FeedShooter;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants.AutoConstants.Auto1Constants;

public class AutoCommandSequence1 extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandSequence1.
   */
  public AutoCommandSequence1(DriveTrain driveTrain, Intake intake, Indexer indexer, Shooter shooter) {
    super(
      new ScheduleCommand(
        new WaitUntilCommand(() -> driveTrain.getDistanceDriven() >= Auto1Constants.INTAKE_START_DISTANCE)
          .andThen(
            new FloorIntake(intake, indexer)
              .withTimeout(Auto1Constants.INTAKE_DURATION)
          )
      ),
      new DriveForward(Auto1Constants.INTAKE_TARGET_DISTANCE, driveTrain), // Drive to line to pick up balls

      new ScheduleCommand( // Index the picked-up balls
        new StartEndCommand(indexer::startIndexer, indexer::stopIndexer)
          .withInterrupt(indexer::ballReady)
      ),
      new TurnDegrees(Auto1Constants.TURN_1_DEGREES, driveTrain),
      new DriveForward(Auto1Constants.DRIVE_1_DISTANCE, driveTrain),
      new TurnDegrees(Auto1Constants.TURN_2_DEGREES, driveTrain),
      new InstantCommand(shooter::enable),
      new DriveForward(Auto1Constants.DRIVE_TO_WALL_DISTANCE, driveTrain)
        .withTimeout(Auto1Constants.MAX_DRIVE_TIME), // In case we are off and never reach

      new InstantCommand(indexer::startIndexer),
      new FeedShooter(shooter, indexer)
        .withTimeout(Auto1Constants.SHOOT_TIME), // Shoot for x seconds
      new InstantCommand(shooter::disable),
      new InstantCommand(indexer::stopIndexer)
    );
  }
}
