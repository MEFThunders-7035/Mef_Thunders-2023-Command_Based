package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonCameraSystem;

public class VisionTargettingCmd extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    private final PIDController fowardController;
    private final PIDController turnController;
    private final PhotonCameraSystem cameraSystem;

    public VisionTargettingCmd(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.cameraSystem = driveSubsystem.getCameraSystem();
        var fowardPID = cameraSystem.cameraDetails.getFowardPIDConstants();
        var turnPID = cameraSystem.cameraDetails.getTurnPIDConstants();
        fowardController = new PIDController(fowardPID.kP, fowardPID.kD, fowardPID.kI);
        turnController = new PIDController(turnPID.kP, turnPID.kD, turnPID.kI);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Vision Targetting Started!");
    }
    
    @Override
    public void execute() {
        var trackedTagets = cameraSystem.getTrackedTargets();
        
        if (trackedTagets.isEmpty()) {
            driveSubsystem.stopMotors();
            return;
        }

        double fowardSpeed = fowardController.calculate(cameraSystem.getArea(), PhotonVisionConstants.kTargetArea);
        double turnSpeed = turnController.calculate(cameraSystem.getYaw(),0);
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
