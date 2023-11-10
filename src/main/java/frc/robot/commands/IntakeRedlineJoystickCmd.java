package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RedlineIntakeSubsystem;

public class IntakeRedlineJoystickCmd extends CommandBase{
    private final double speed;
    private final RedlineIntakeSubsystem intakeSubsystem;

    public IntakeRedlineJoystickCmd(RedlineIntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        // we don't need to initialize anything 
        // because we drive with joysticks
    }

    @Override
    public void execute() {
        intakeSubsystem.setRedline(speed);
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRedline(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
