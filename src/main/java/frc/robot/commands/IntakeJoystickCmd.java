package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
public class IntakeJoystickCmd extends CommandBase{
    private final double speed;
    private final IntakeSubsystem Intakesubsystem;
    public IntakeJoystickCmd(IntakeSubsystem Intakesubsystem, double speed) {
        this.Intakesubsystem = Intakesubsystem;
        this.speed = speed;
        addRequirements(Intakesubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Intakesubsystem.setNeo(speed);
    }
    @Override
    public void end(boolean interrupted) {
        Intakesubsystem.setNeo(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
