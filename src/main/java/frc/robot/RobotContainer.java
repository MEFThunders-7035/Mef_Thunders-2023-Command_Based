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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.PhotonCameraSystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.RedlineIntakeSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;



public class RobotContainer {
  private final Field2d field2d = new Field2d();
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(field2d);
  private final VerticalElevatorSubsystem verticalElevatorSubsystem = new VerticalElevatorSubsystem();
  private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
  private final RedlineIntakeSubsystem redlineIntakeSubsystem = new RedlineIntakeSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> cameraChooser = new SendableChooser<>();
  private final Joystick stick = new Joystick(OperatorConstants.kJoystickPort);
  private String autoSelected;
  
  public RobotContainer() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    configureBindings();
    addChoosers();
    setupPhotonVisionCamera();
    verticalElevatorSubsystem.setDefaultCommand(new VerticalElevatorJoystickCmd(verticalElevatorSubsystem, 0));
    intakeArmSubsystem.setDefaultCommand(new HoldIntakeCmd(intakeArmSubsystem));
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> stick.getRawAxis(IoConstants.Y_AXIS), () -> stick.getRawAxis(IoConstants.Z_AXIS)));
  }

  public void fastPeriodic() {
    double first = Timer.getFPGATimestamp();
    driveSubsystem.runGyroLoop();
    double timeTook = Timer.getFPGATimestamp() - first;
    if (timeTook >= 0.1) {
      DriverStation.reportWarning("Loop Time of 0.01 Overrun, Time Took: " + timeTook, false);
    } 
  }

  private void configureBindings() {
    new POVButton(stick, 0).whileTrue(new VerticalElevatorJoystickCmd(verticalElevatorSubsystem, VerticalElevatorConstants.kSpeed).until(verticalElevatorSubsystem.getTopLimitSwitchSupplier()));
    new POVButton(stick, 180).whileTrue(new VerticalElevatorJoystickCmd(verticalElevatorSubsystem, -VerticalElevatorConstants.kSpeed).until(verticalElevatorSubsystem.getBottomLimitSwitchSupplier()));
    new JoystickButton(stick, 3).whileTrue(new IntakeNeoJoystickCmd(intakeArmSubsystem, IntakeConstants.kUpSpeed));
    new JoystickButton(stick, 4).whileTrue(new IntakeNeoJoystickCmd(intakeArmSubsystem, IntakeConstants.kDownSpeed));
    new JoystickButton(stick, 5).whileTrue(new IntakeRedlineJoystickCmd(redlineIntakeSubsystem, IntakeConstants.kRedlineSpeed));
    new JoystickButton(stick, 6).whileTrue(new IntakeRedlineJoystickCmd(redlineIntakeSubsystem, -IntakeConstants.kRedlineSpeed));
    new JoystickButton(stick, 7).whileTrue(new SetSelenoidsCmd(pneumaticsSubsystem, false));
    new JoystickButton(stick, 8).whileTrue(new SetSelenoidsCmd(pneumaticsSubsystem, true));
  }

  private void addChoosers() {
    autoChooser.setDefaultOption("Timer Auto", AutonomousConstants.kTimedAuto);
    autoChooser.addOption("Camera Auto", AutonomousConstants.kCameraAuto);
    autoChooser.addOption("Stabilize Auto", AutonomousConstants.kStabilize);
    autoChooser.addOption("Ramsete Auto", AutonomousConstants.kRamsete);
    autoChooser.addOption("Encoder Drive Auto", AutonomousConstants.kEncoder);
    autoChooser.addOption("Path Follow Auto", AutonomousConstants.kPath);
    SmartDashboard.putData("Auto choices", autoChooser);

    cameraChooser.setDefaultOption("Pi Cam", PhotonVisionConstants.Cameras.kPiCamera);
    cameraChooser.addOption("Wide Cam", PhotonVisionConstants.Cameras.kWideCamera);
    SmartDashboard.putData("Camera choices", cameraChooser);

  }

  private void setupPhotonVisionCamera() {
    String cameraSelected = cameraChooser.getSelected();
    switch (cameraSelected) {
      case PhotonVisionConstants.Cameras.kPiCamera:
        driveSubsystem.setCameraSystem(new PhotonCameraSystem(new PhotonVisionConstants.New_PiCamera()));
        break;
      case PhotonVisionConstants.Cameras.kWideCamera:
        driveSubsystem.setCameraSystem(new PhotonCameraSystem(new PhotonVisionConstants.New_WideCamera()));
        break;
      default:
        DriverStation.reportError("Unknown Camera Selected: " + cameraSelected, false);
    }
  }

  public Command getAutonomousCommand() {
    autoSelected = autoChooser.getSelected();

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
      new TimedIntakeRedlineCmd(redlineIntakeSubsystem, AutonomousConstants.kRedlineSpeed, AutonomousConstants.kRedlineTime),
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
