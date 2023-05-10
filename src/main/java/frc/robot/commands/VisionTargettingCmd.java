package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
public class VisionTargettingCmd extends CommandBase{
    private final PhotonVisionSubsystem photonVisionSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final PIDController FowardController;
    private final PIDController TurnController;
    private double fowardSpeed;
    private double turnSpeed;

    public VisionTargettingCmd(PhotonVisionSubsystem photonVisionSubsystem, DriveSubsystem driveSubsystem) {
        this.photonVisionSubsystem = photonVisionSubsystem;
        this.driveSubsystem = driveSubsystem;
        FowardController = new PIDController(PhotonVisionConstants.FowardPIDConstants.kP, PhotonVisionConstants.FowardPIDConstants.kI, PhotonVisionConstants.FowardPIDConstants.kD);
        TurnController = new PIDController(PhotonVisionConstants.TurnPIDConstants.kP, PhotonVisionConstants.TurnPIDConstants.kI, PhotonVisionConstants.TurnPIDConstants.kD);
        addRequirements(photonVisionSubsystem);
        addRequirements(driveSubsystem);

        FowardController.setIntegratorRange(0.1, 0.4);
        TurnController.setIntegratorRange(0.1, 0.5);
    }

    @Override
    public void initialize() {
        System.out.println("Vision Targetting Started!");
    }
    
    @Override
    public void execute() {
        if (!photonVisionSubsystem.hasTargets()) {
            driveSubsystem.stop();
            return;
        }

        fowardSpeed = FowardController.calculate(photonVisionSubsystem.getArea(),PhotonVisionConstants.kTargetArea);
        turnSpeed = TurnController.calculate(photonVisionSubsystem.getYaw(),0);
        driveSubsystem.drive(-fowardSpeed, -turnSpeed, false);
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
