package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmSubsystem;
public class IntakeNeoJoystickCmd extends CommandBase{
    private final double speed;
    private final IntakeArmSubsystem Intakesubsystem;
    public IntakeNeoJoystickCmd(IntakeArmSubsystem Intakesubsystem, double speed) {
        this.Intakesubsystem = Intakesubsystem;
        this.speed = speed;
        addRequirements(Intakesubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Intakesubsystem.setMotors(speed);
    }
    @Override
    public void end(boolean interrupted) {
        Intakesubsystem.setMotors(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
