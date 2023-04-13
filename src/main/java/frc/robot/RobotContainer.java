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
import frc.robot.commands.IntakeNeoJoystickCmd;
import frc.robot.commands.IntakeRedlineJoystickCmd;
import frc.robot.commands.VerticalElevatorJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HorizontalElevatorSubsytem;
import frc.robot.subsystems.Neo_IntakeSubsystem;
import frc.robot.subsystems.Redline_IntakeSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final VerticalElevatorSubsystem vertical_ElevatorSubsytem = new VerticalElevatorSubsystem();
  private final HorizontalElevatorSubsytem horizontal_ElevatorSubsystem = new HorizontalElevatorSubsytem();
  private final Neo_IntakeSubsystem Neo_IntakeSubsystem = new Neo_IntakeSubsystem();
  private final Redline_IntakeSubsystem Redline_IntakeSubsystem = new Redline_IntakeSubsystem();

  private final Joystick stick = new Joystick(OperatorConstants.kJoystickPort);
  public RobotContainer() {
    configureBindings();
    vertical_ElevatorSubsytem.setDefaultCommand(new VerticalElevatorJoystickCmd(vertical_ElevatorSubsytem, 0));
    horizontal_ElevatorSubsystem.setDefaultCommand(new HorizontalElevatorJoystickCmd(horizontal_ElevatorSubsystem, 0));
    Neo_IntakeSubsystem.setDefaultCommand(new HoldIntakeCmd(Neo_IntakeSubsystem));
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> stick.getRawAxis(IoConstants.Y_AXIS), () -> stick.getRawAxis(IoConstants.Z_AXIS)));
  }

  private void configureBindings() {
    new POVButton(stick, 0).whileTrue(new HorizontalElevatorJoystickCmd(horizontal_ElevatorSubsystem, HorizontalElevatorConstants.kSpeed));
    new POVButton(stick, 180).whileTrue(new HorizontalElevatorJoystickCmd(horizontal_ElevatorSubsystem, -HorizontalElevatorConstants.kSpeed));
    new JoystickButton(stick, 1).whileTrue(new VerticalElevatorJoystickCmd(vertical_ElevatorSubsytem, VerticalElevatorConstants.kSpeed));
    new JoystickButton(stick, 2).whileTrue(new VerticalElevatorJoystickCmd(vertical_ElevatorSubsytem, -VerticalElevatorConstants.kSpeed));
    new JoystickButton(stick, 3).whileTrue(new IntakeNeoJoystickCmd(Neo_IntakeSubsystem, IntakeConstants.kUpSpeed));
    new JoystickButton(stick, 4).whileTrue(new IntakeNeoJoystickCmd(Neo_IntakeSubsystem, IntakeConstants.kDownSpeed));
    new JoystickButton(stick, 5).whileTrue(new IntakeRedlineJoystickCmd(Redline_IntakeSubsystem, IntakeConstants.kRedlineSpeed));
    new JoystickButton(stick, 6).whileTrue(new IntakeRedlineJoystickCmd(Redline_IntakeSubsystem, -IntakeConstants.kRedlineSpeed));
  }
}
