package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmSubsystem;
public class IntakeNeoJoystickCmd extends CommandBase{
    private final double speed;
    private final IntakeArmSubsystem intakeSubsystem;
    public IntakeNeoJoystickCmd(IntakeArmSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        // we don't need to initialize anything
    }

    @Override
    public void execute() {
        intakeSubsystem.setMotors(speed);
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotors(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
