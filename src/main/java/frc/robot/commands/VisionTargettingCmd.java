package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
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
        FowardController = new PIDController(photonVisionSubsystem.getCurrentFowardPIDConstants()[0], photonVisionSubsystem.getCurrentFowardPIDConstants()[1], photonVisionSubsystem.getCurrentFowardPIDConstants()[2]);
        TurnController = new PIDController(photonVisionSubsystem.getCurrentTurnPIDConstants()[0], photonVisionSubsystem.getCurrentTurnPIDConstants()[1], photonVisionSubsystem.getCurrentTurnPIDConstants()[2]);
        addRequirements(photonVisionSubsystem);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Vision Targetting Started!");
    }
    
    @Override
    public void execute() {
        if (!photonVisionSubsystem.hasTargets()) {
            driveSubsystem.stopMotors();
            return;
        }
        try {
            fowardSpeed = FowardController.calculate(photonVisionSubsystem.getArea(),PhotonVisionConstants.kTargetArea);
            turnSpeed = TurnController.calculate(photonVisionSubsystem.getYaw(),0);
            driveSubsystem.drive(-fowardSpeed, -turnSpeed, false);
        }
        catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
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
