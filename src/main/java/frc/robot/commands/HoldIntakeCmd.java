package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeArmSubsystem;


public class HoldIntakeCmd extends CommandBase{
    private final IntakeArmSubsystem IntakeArmsubsystem;
    private final PIDController pos_pid = new PIDController(IntakeConstants.IntakePIDConstants.kP, IntakeConstants.IntakePIDConstants.kI, IntakeConstants.IntakePIDConstants.kD);

    public HoldIntakeCmd(IntakeArmSubsystem Intakesubsystem) {
        this.IntakeArmsubsystem = Intakesubsystem;
        addRequirements(Intakesubsystem);
    }
    @Override
    public void initialize() {
        pos_pid.reset();
        pos_pid.setSetpoint(IntakeArmsubsystem.getArmPos());
    }
    @Override
    public void execute() {
        double speed = pos_pid.calculate(IntakeArmsubsystem.getArmPos());
        IntakeArmsubsystem.setMotors(speed);
    }
    @Override
    public void end(boolean interrupted) {
        IntakeArmsubsystem.setMotors(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }    
}
