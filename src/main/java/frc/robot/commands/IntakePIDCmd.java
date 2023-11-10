package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeArmSubsystem;
public class IntakePIDCmd extends CommandBase{
    private final IntakeArmSubsystem intakeSubsystem;
    private final PIDController pid = new PIDController(IntakeConstants.IntakePIDConstants.kP, IntakeConstants.IntakePIDConstants.kI, IntakeConstants.IntakePIDConstants.kD);
    public IntakePIDCmd(IntakeArmSubsystem intakeSubsystem, double setpoint) {
        this.intakeSubsystem = intakeSubsystem;
        pid.setSetpoint(setpoint);
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        pid.reset();
    }
    @Override
    public void execute() {
        double speed = pid.calculate(intakeSubsystem.getArmPos());
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
