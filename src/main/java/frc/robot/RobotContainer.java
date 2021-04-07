// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  public Joystick leftStick = new Joystick(0);
  public Joystick rightStick = new Joystick(1);


  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();

  private final DriveWithArcadeCommand driveWithArcadeCommand = new DriveWithArcadeCommand(driveTrainSubsystem, rightStick);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  JoystickButton leftTrigger = new JoystickButton(leftStick, 1);
  JoystickButton leftThumbMain = new JoystickButton(leftStick, 2);
  JoystickButton leftThumbLeft = new JoystickButton(leftStick, 3);
  JoystickButton leftThumbRight = new JoystickButton(leftStick, 4);
  JoystickButton leftRightArrayTR = new JoystickButton(leftStick, 5);
  JoystickButton leftRightArrayTM = new JoystickButton(leftStick, 6);
  JoystickButton leftRightArrayTL = new JoystickButton(leftStick, 7);
  JoystickButton leftRightArrayBL = new JoystickButton(leftStick, 8);
  JoystickButton leftRightArrayBM = new JoystickButton(leftStick, 9);
  JoystickButton leftRightArrayBR = new JoystickButton(leftStick, 10);
  JoystickButton leftLeftArrayTL = new JoystickButton(leftStick, 11);
  JoystickButton leftLeftArrayTM = new JoystickButton(leftStick, 12);
  JoystickButton leftLeftArrayTR = new JoystickButton(leftStick, 13);
  JoystickButton leftLeftArrayBR = new JoystickButton(leftStick, 14);
  JoystickButton leftLeftArrayBM = new JoystickButton(leftStick, 15);
  JoystickButton leftLeftArrayBL = new JoystickButton(leftStick, 16);

  JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
  JoystickButton rightThumbMain = new JoystickButton(rightStick, 2);
  JoystickButton rightThumbLeft = new JoystickButton(rightStick, 3);
  JoystickButton rightThumbRight = new JoystickButton(rightStick, 4);
  JoystickButton rightRightArrayTR = new JoystickButton(rightStick, 5);
  JoystickButton rightRightArrayTM = new JoystickButton(rightStick, 6);
  JoystickButton rightRightArrayTL = new JoystickButton(rightStick, 7);
  JoystickButton rightRightArrayBL = new JoystickButton(rightStick, 8);
  JoystickButton rightRightArrayBM = new JoystickButton(rightStick, 9);
  JoystickButton rightRightArrayBR = new JoystickButton(rightStick, 10);
  JoystickButton rightLeftArrayTL = new JoystickButton(rightStick, 11);
  JoystickButton rightLeftArrayTM = new JoystickButton(rightStick, 12);
  JoystickButton rightLeftArrayTR = new JoystickButton(rightStick, 13);
  JoystickButton rightLeftArrayBR = new JoystickButton(rightStick, 14);
  JoystickButton rightLeftArrayBM = new JoystickButton(rightStick, 15);
  JoystickButton rightLeftArrayBL = new JoystickButton(rightStick, 16);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureButtonBindings();
    driveTrainSubsystem.setDefaultCommand(driveWithArcadeCommand);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    rightThumbRight
      .whenPressed(new DriveWithJoysticksCommand(driveTrainSubsystem, leftStick, leftStick));
    rightThumbLeft
      .whenPressed(new DriveWithArcadeCommand(driveTrainSubsystem, leftStick));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
