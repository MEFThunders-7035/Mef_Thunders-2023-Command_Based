package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
public class VisionTargettingCmd extends CommandBase{
    private final PhotonVisionSubsystem photonVisionSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final PIDController FowardController = new PIDController(PhotonVisionConstants.FowardPIDConstants.kP, PhotonVisionConstants.FowardPIDConstants.kI, PhotonVisionConstants.FowardPIDConstants.kD);
    private final PIDController TurnController = new PIDController(PhotonVisionConstants.TurnPIDConstants.kP, PhotonVisionConstants.TurnPIDConstants.kI, PhotonVisionConstants.TurnPIDConstants.kD);
    private double fowardSpeed;
    private double turnSpeed;
    public VisionTargettingCmd(PhotonVisionSubsystem photonVisionSubsystem, DriveSubsystem driveSubsystem) {
        this.photonVisionSubsystem = photonVisionSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(photonVisionSubsystem);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        fowardSpeed = FowardController.calculate(photonVisionSubsystem.getDistance(),0);
        turnSpeed = TurnController.calculate(photonVisionSubsystem.getYaw(),0);
        driveSubsystem.drive(fowardSpeed, turnSpeed);
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
