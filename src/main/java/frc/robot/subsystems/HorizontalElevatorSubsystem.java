package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VerticalElevatorConstants;


public class HorizontalElevatorSubsystem extends SubsystemBase{

    private final Spark Vertical_Elevator = new Spark(VerticalElevatorConstants.kVerticalElevatorPort);
    
    public HorizontalElevatorSubsystem() {
        Vertical_Elevator.setInverted(true);
    }

    public void periodic() {
        if (!Vertical_Elevator.isAlive()) {
            DriverStation.reportError("Vertical Elevator motor is not alive", false);
            throw new RuntimeException("Vertical Elevator motor is not alive");
        }
    }
    
    public void setMotor(double speed) {
        Vertical_Elevator.set(speed);
    }
}
