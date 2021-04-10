// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;

import frc.robot.commands.ResetEncoderCommand;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExampleRamseteCommandGroup extends SequentialCommandGroup {
  Trajectory trajectory = new Trajectory();
  
  /** Creates a new BarrelRacingPath. */
  public ExampleRamseteCommandGroup(DriveTrainSubsystem driveTrainSubsystem) {
    String trajectoryJSON = "paths/output/barrelRacingPath3.wpilib.json";
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    /*RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      driveTrainSubsystem::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(KS,
                                KV,
                                KA),
                                K_DRIVE_KINEMATICS,
      driveTrainSubsystem::getWheelSpeeds,
      new PIDController(KP, 0, 0),
      new PIDController(KP, 0, 0),
      // RamseteCommand passes volts to the callback
      driveTrainSubsystem::tankDriveVolts,
      driveTrainSubsystem
      );
      */

      
      //ActivateIntakeCommand activateIntakeCommand = new ActivateIntakeCommand(intakeSubsystem);

      ResetOdometryCommand resetOdometryCommand = new ResetOdometryCommand(driveTrainSubsystem, trajectory);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(resetOdometryCommand);
  }
}
