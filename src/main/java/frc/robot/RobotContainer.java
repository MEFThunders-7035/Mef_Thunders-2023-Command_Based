// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.VerticalElevatorConstants;

import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.EncoderDriveCmd;
import frc.robot.commands.HoldIntakeCmd;
import frc.robot.commands.IntakeNeoJoystickCmd;
import frc.robot.commands.IntakeRedlineJoystickCmd;
import frc.robot.commands.SetSelenoidsCmd;
import frc.robot.commands.TimedDriveCmd;
import frc.robot.commands.TimedIntakeRedlineCmd;
import frc.robot.commands.VerticalElevatorJoystickCmd;
import frc.robot.commands.VisionTargettingCmd;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.Redline_IntakeSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;



public class RobotContainer {
  private final Field2d field2d = new Field2d();
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(field2d);
  private final VerticalElevatorSubsystem Vertical_Elevator_Subsytem = new VerticalElevatorSubsystem();
  private final IntakeArmSubsystem IntakeArmSubsystem = new IntakeArmSubsystem();
  private final Redline_IntakeSubsystem Redline_IntakeSubsystem = new Redline_IntakeSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  private final SendableChooser<String> Auto_chooser = new SendableChooser<>();
  private final SendableChooser<String> Camera_chooser = new SendableChooser<>();
  private final Joystick stick = new Joystick(OperatorConstants.kJoystickPort);
  private String autoSelected;
  
  public RobotContainer() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    configureBindings();
    AddChoosers();
    setupPhotonVisionCamera();
    Vertical_Elevator_Subsytem.setDefaultCommand(new VerticalElevatorJoystickCmd(Vertical_Elevator_Subsytem, 0));
    IntakeArmSubsystem.setDefaultCommand(new HoldIntakeCmd(IntakeArmSubsystem));
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> stick.getRawAxis(IoConstants.Y_AXIS), () -> stick.getRawAxis(IoConstants.Z_AXIS)));
  }

  public void fast_periodic() {
    double first = Timer.getFPGATimestamp();
    driveSubsystem.runGyroLoop();
    double time_took = Timer.getFPGATimestamp() - first;
    if (time_took >= 0.1) {
      DriverStation.reportWarning("Loop Time of 0.01 Overrun, Time Took: " + time_took, false);
    } 
  }

  private void configureBindings() {
    new POVButton(stick, 0).whileTrue(new VerticalElevatorJoystickCmd(Vertical_Elevator_Subsytem, VerticalElevatorConstants.kSpeed).until(Vertical_Elevator_Subsytem.getTopLimitSwitchSupplier()));
    new POVButton(stick, 180).whileTrue(new VerticalElevatorJoystickCmd(Vertical_Elevator_Subsytem, -VerticalElevatorConstants.kSpeed).until(Vertical_Elevator_Subsytem.getBottomLimitSwitchSupplier()));
    new JoystickButton(stick, 3).whileTrue(new IntakeNeoJoystickCmd(IntakeArmSubsystem, IntakeConstants.kUpSpeed));
    new JoystickButton(stick, 4).whileTrue(new IntakeNeoJoystickCmd(IntakeArmSubsystem, IntakeConstants.kDownSpeed));
    new JoystickButton(stick, 5).whileTrue(new IntakeRedlineJoystickCmd(Redline_IntakeSubsystem, IntakeConstants.kRedlineSpeed));
    new JoystickButton(stick, 6).whileTrue(new IntakeRedlineJoystickCmd(Redline_IntakeSubsystem, -IntakeConstants.kRedlineSpeed));
    new JoystickButton(stick, 7).whileTrue(new SetSelenoidsCmd(pneumaticsSubsystem, false));
    new JoystickButton(stick, 8).whileTrue(new SetSelenoidsCmd(pneumaticsSubsystem, true));
  }

  private void AddChoosers() {
    Auto_chooser.setDefaultOption("Timer Auto", AutonomousConstants.kTimedAuto);
    Auto_chooser.addOption("Camera Auto", AutonomousConstants.kCameraAuto);
    Auto_chooser.addOption("Stabilize Auto", AutonomousConstants.kStabilize);
    Auto_chooser.addOption("Ramsete Auto", AutonomousConstants.kRamsete);
    Auto_chooser.addOption("Encoder Drive Auto", AutonomousConstants.kEncoder);
    Auto_chooser.addOption("Path Follow Auto", AutonomousConstants.kPath);
    SmartDashboard.putData("Auto choices", Auto_chooser);

    Camera_chooser.setDefaultOption("Pi Cam", PhotonVisionConstants.Cameras.kPiCamera);
    Camera_chooser.addOption("Wide Cam", PhotonVisionConstants.Cameras.kWideCamera);
    SmartDashboard.putData("Camera choices", Camera_chooser);

  }

  private void setupPhotonVisionCamera() {
    String cameraSelected = Camera_chooser.getSelected();
  }

  public Command getAutonomousCommand() {
    autoSelected = Auto_chooser.getSelected();

    switch (autoSelected) {
      case AutonomousConstants.kTimedAuto:
        return timedAuto();
      case AutonomousConstants.kCameraAuto:
        return cameraAuto();
      case AutonomousConstants.kStabilize:
        return stabilizeAuto();
      case AutonomousConstants.kRamsete:
        return ramseteCommand();
      case AutonomousConstants.kEncoder:
        return EncoderDriveAutoCommand();
      case AutonomousConstants.kPath:
        return pathFollowCommand();
      default:
        return timedAuto();
    }
  }
  
  private Command timedAuto() {
    return new SequentialCommandGroup(
      new TimedIntakeRedlineCmd(Redline_IntakeSubsystem, AutonomousConstants.kRedlineSpeed, AutonomousConstants.kRedlineTime),
      new TimedDriveCmd(driveSubsystem, AutonomousConstants.kDriveSpeed, AutonomousConstants.kDriveTime)
    );
  }
  
  private Command cameraAuto() {
    return null;
  }
  
  private Command stabilizeAuto() {
    return null;
  }

  private Command EncoderDriveAutoCommand() {
    return new EncoderDriveCmd(driveSubsystem, AutonomousConstants.kDriveAmount);
  }

  private Command ramseteCommand() {
    var VoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.FeedForwardConstants.ksVolts,
        DriveConstants.FeedForwardConstants.kvVoltSecondsPerMeter,
        DriveConstants.FeedForwardConstants.kaVoltSecondsSquaredPerMeter), 
        DriveConstants.kDriveKinematics,
        8);
    
    TrajectoryConfig config =
    new TrajectoryConfig(
      DriveConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
    // Apply the voltage constraint
    .addConstraint(VoltageConstraint);
    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, going foward at 2 m/s
      List.of(
        new Translation2d(1, 0)
      ),
      // End 1 meters straight ahead of where we started, facing forward
      new Pose2d(2, 0, new Rotation2d(0)),
      // Pass config
      config
    );
    
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            driveSubsystem.getPose2dSupplier(),
            new RamseteController(),
            new SimpleMotorFeedforward(
                DriveConstants.FeedForwardConstants.ksVolts,
                DriveConstants.FeedForwardConstants.kvVoltSecondsPerMeter,
                DriveConstants.FeedForwardConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveSubsystem::setMotorVoltage,
            driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveSubsystem.setMotorVoltage(0, 0)).andThen(() -> System.out.println("Ramsete Command Finished"));
  }

  private Command pathFollowCommand() {
    return driveSubsystem.pathFollowCommand();
  }
}
