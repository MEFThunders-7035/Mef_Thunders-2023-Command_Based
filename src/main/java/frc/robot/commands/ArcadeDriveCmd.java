package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCmd extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> speedfunc;
    private final Supplier<Double> turnfunc;
    public ArcadeDriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> speedfunc, Supplier<Double> turnfunc) {
        this.driveSubsystem = driveSubsystem;
        this.speedfunc = speedfunc;
        this.turnfunc = turnfunc;
        addRequirements(driveSubsystem);
    }
    @Override
    public void execute() {
        driveSubsystem.drive(speedfunc.get(), turnfunc.get());
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
