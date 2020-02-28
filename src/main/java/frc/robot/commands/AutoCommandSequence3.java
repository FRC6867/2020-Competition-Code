/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

import frc.robot.commands.CollectFromIntake;
import frc.robot.commands.Shoot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants.AutoConstants.Auto3Constants;

public class AutoCommandSequence3 extends SequentialCommandGroup {
  /**
   * Creates a new AutoCommandSequence1.
   */
  public AutoCommandSequence3(DriveTrain driveTrain, Indexer indexer, Shooter shooter) {
    super(
      new InstantCommand(shooter::enable),
      new WaitCommand(Auto3Constants.SPIN_UP_TIME),
      new ScheduleCommand(
        new Shoot(shooter, indexer)
          .withTimeout(20) // Shoot for x seconds
      ),
      new StartEndCommand(indexer::startIndexer, indexer::vomit)
        .withTimeout(1.5),
      new WaitCommand(0.5),
      new StartEndCommand(indexer::startIndexer, indexer::vomit)
        .withTimeout(1.5),
      new WaitCommand(0.5),
      new StartEndCommand(indexer::startIndexer, indexer::vomit)
        .withTimeout(1.5),
      new WaitCommand(0.5),
      new StartEndCommand(indexer::startIndexer, indexer::vomit)
        .withTimeout(1.5),
      new WaitCommand(0.5),
      new StartEndCommand(indexer::startIndexer, indexer::vomit)
        .withTimeout(1.5),
      new WaitCommand(0.5),
      new StartEndCommand(indexer::startIndexer, indexer::vomit)
        .withTimeout(1.5),
      new WaitCommand(0.5),
      new StartEndCommand(indexer::startIndexer, indexer::vomit)
        .withTimeout(1.5),
      new WaitCommand(0.5),
      new InstantCommand(indexer::startIndexer),
      new WaitCommand(1.5),
      new InstantCommand(indexer::stopIndexer),

      new InstantCommand(shooter::disable),
      new InstantCommand(() -> driveTrain.driveStraight(-1), driveTrain),
      new WaitCommand(1),
      new InstantCommand(() -> driveTrain.driveStraight(0), driveTrain)
    );
  }
}
