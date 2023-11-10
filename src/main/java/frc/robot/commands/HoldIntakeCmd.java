package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeArmSubsystem;


public class HoldIntakeCmd extends CommandBase{
    private final IntakeArmSubsystem intakeArmsubsystem;
    private final PIDController drivePID = new PIDController(IntakeConstants.IntakePIDConstants.kP, IntakeConstants.IntakePIDConstants.kI, IntakeConstants.IntakePIDConstants.kD);

    public HoldIntakeCmd(IntakeArmSubsystem intakesubsystem) {
        this.intakeArmsubsystem = intakesubsystem;
        addRequirements(intakesubsystem);
    }
    @Override
    public void initialize() {
        drivePID.reset();
        drivePID.setSetpoint(intakeArmsubsystem.getArmPos());
    }
    @Override
    public void execute() {
        double speed = drivePID.calculate(intakeArmsubsystem.getArmPos());
        intakeArmsubsystem.setMotors(speed);
    }
    @Override
    public void end(boolean interrupted) {
        intakeArmsubsystem.setMotors(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }    
}
