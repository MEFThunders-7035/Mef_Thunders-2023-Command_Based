package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Redline_IntakeSubsystem;

public class IntakeRedlineJoystickCmd extends CommandBase{
    private final double speed;
    private final Redline_IntakeSubsystem Intakesubsystem;

    public IntakeRedlineJoystickCmd(Redline_IntakeSubsystem Intakesubsystem, double speed) {
        this.Intakesubsystem = Intakesubsystem;
        this.speed = speed;
        addRequirements(Intakesubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Intakesubsystem.setRedline(speed);
    }
    @Override
    public void end(boolean interrupted) {
        Intakesubsystem.setRedline(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
