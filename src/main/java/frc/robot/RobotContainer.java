// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VerticalElevatorConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.HoldIntakeCmd;
import frc.robot.commands.VerticalElevatorJoystickCmd;
import frc.robot.commands.IntakeJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HorizontalElevatorSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final VerticalElevatorSubsystem verticalElvElevatorSubsytem = new VerticalElevatorSubsystem();
  private final HorizontalElevatorSubsytem horizontalElevatorSubsystem = new HorizontalElevatorSubsytem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final Joystick stick = new Joystick(OperatorConstants.kJoystickPort);
  public RobotContainer() {
    configureBindings();
    verticalElvElevatorSubsytem.setDefaultCommand(new VerticalElevatorJoystickCmd(verticalElvElevatorSubsytem, 0));
    horizontalElevatorSubsystem.setDefaultCommand(null);
    intakeSubsystem.setDefaultCommand(new HoldIntakeCmd(intakeSubsystem));
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> stick.getRawAxis(1), () -> stick.getRawAxis(3)));
  }

  private void configureBindings() {
    new JoystickButton(stick, 1).whileTrue(new VerticalElevatorJoystickCmd(verticalElvElevatorSubsytem, VerticalElevatorConstants.kSpeed));
    new JoystickButton(stick, 2).whileTrue(new VerticalElevatorJoystickCmd(verticalElvElevatorSubsytem, -VerticalElevatorConstants.kSpeed));
    new JoystickButton(stick, 3).whileTrue(new IntakeJoystickCmd(intakeSubsystem, IntakeConstants.kUpSpeed));
    new JoystickButton(stick, 4).whileTrue(new IntakeJoystickCmd(intakeSubsystem, IntakeConstants.kDownSpeed));
  }
}
