package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonCameraSystem;

public class VisionTargettingCmd extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    private final PIDController FowardController;
    private final PIDController TurnController;
    private final PhotonCameraSystem CameraSystem;
    private double fowardSpeed;
    private double turnSpeed;

    public VisionTargettingCmd(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.CameraSystem = driveSubsystem.getCameraSystem();
        var fowardPID = CameraSystem.camera_details.getFowardPIDConstants();
        var turnPID = CameraSystem.camera_details.getTurnPIDConstants();
        FowardController = new PIDController(fowardPID.kP, fowardPID.kD, fowardPID.kI);
        TurnController = new PIDController(turnPID.kP, turnPID.kD, turnPID.kI);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Vision Targetting Started!");
    }
    
    @Override
    public void execute() {
        var tracked_tagets = CameraSystem.getTrackedTargets();
        
        if (tracked_tagets.isEmpty()) {
            driveSubsystem.stopMotors();
            return;
        }

        fowardSpeed = FowardController.calculate(CameraSystem.getArea(), PhotonVisionConstants.kTargetArea);
        turnSpeed = TurnController.calculate(CameraSystem.getYaw(),0);
        driveSubsystem.drive(fowardSpeed, turnSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
