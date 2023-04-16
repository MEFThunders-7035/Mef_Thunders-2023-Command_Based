package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCmd extends CommandBase{
    private final DriveSubsystem Drivesubsystem;
    private final Supplier<Double> speedfunc, turnfunc;
    public ArcadeDriveCmd(DriveSubsystem Drivesubsystem, Supplier<Double> speedfunc, Supplier<Double> turnfunc) {
        this.Drivesubsystem = Drivesubsystem;
        this.speedfunc = speedfunc;
        this.turnfunc = turnfunc;
        addRequirements(Drivesubsystem);
    }
    @Override
    public void execute() {
        Drivesubsystem.drive(-speedfunc.get(), -turnfunc.get());
    }
    @Override
    public void end(boolean interrupted) {
        Drivesubsystem.drive(0, 0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
