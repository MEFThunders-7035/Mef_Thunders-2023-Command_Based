package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Neo_IntakeSubsystem;
public class IntakeNeoJoystickCmd extends CommandBase{
    private final double speed;
    private final Neo_IntakeSubsystem Intakesubsystem;
    public IntakeNeoJoystickCmd(Neo_IntakeSubsystem Intakesubsystem, double speed) {
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
