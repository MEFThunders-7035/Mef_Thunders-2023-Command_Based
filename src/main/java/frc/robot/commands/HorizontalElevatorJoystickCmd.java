package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalElevatorSubsystem;
public class HorizontalElevatorJoystickCmd extends CommandBase{
    private final HorizontalElevatorSubsystem horizontalElevatorSubsytem;
    private final double speed;
    public HorizontalElevatorJoystickCmd(HorizontalElevatorSubsystem horizontalElevatorSubsytem, double speed) {
        this.horizontalElevatorSubsytem = horizontalElevatorSubsytem;
        this.speed = speed;
        addRequirements(horizontalElevatorSubsytem);
    }
    @Override
    public void execute() {
        horizontalElevatorSubsytem.setMotor(speed);
    }
    @Override
    public void end(boolean interrupted) {
        horizontalElevatorSubsytem.setMotor(0);
    }
   
    @Override
    public boolean isFinished() {
        return false;
    }
}
