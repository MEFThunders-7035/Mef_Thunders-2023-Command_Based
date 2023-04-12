package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalElevatorSubsytem;
public class HorizontalElevatorJoystickCmd extends CommandBase{
    private final HorizontalElevatorSubsytem horizontalElevatorSubsytem;
    private final double speed;
    public HorizontalElevatorJoystickCmd(HorizontalElevatorSubsytem horizontalElevatorSubsytem, double speed) {
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
