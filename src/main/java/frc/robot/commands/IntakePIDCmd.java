package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeArmSubsystem;
public class IntakePIDCmd extends CommandBase{
    private final IntakeArmSubsystem Intakesubsystem;
    private final PIDController pid = new PIDController(IntakeConstants.IntakePIDConstants.kP, IntakeConstants.IntakePIDConstants.kI, IntakeConstants.IntakePIDConstants.kD);
    public IntakePIDCmd(IntakeArmSubsystem Intakesubsystem, double setpoint) {
        this.Intakesubsystem = Intakesubsystem;
        pid.setSetpoint(setpoint);
        addRequirements(Intakesubsystem);
    }
    @Override
    public void initialize() {
        pid.reset();
    }
    @Override
    public void execute() {
        double speed = pid.calculate(Intakesubsystem.getMotorsPos());
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
