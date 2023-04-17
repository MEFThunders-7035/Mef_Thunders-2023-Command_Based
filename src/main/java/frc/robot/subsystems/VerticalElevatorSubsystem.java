package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HorizontalElevatorConstants;

public class VerticalElevatorSubsystem extends SubsystemBase{

    public static final Spark Elevator = new Spark(HorizontalElevatorConstants.kHorizontalElevatorPort);
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);


    public VerticalElevatorSubsystem() {
        Elevator.setInverted(true);
    }
    
    

        if (!Elevator.isAlive()) {
            DriverStation.reportError("Elevator motor is not alive", false);
            throw new RuntimeException("Elevator motor is not alive");
        }
    }

    public void setMotor(double speed) {
        Elevator.set(speed);
    }
    
    public boolean getTopLimitSwitch() {
        return !toplimitSwitch.get();
    }

    public BooleanSupplier getTopLimitSwitchSupplier() {
        return () -> getTopLimitSwitch();
    }
    
    public boolean getBottomLimitSwitch() {
        return !bottomlimitSwitch.get();
    }
    
    public BooleanSupplier getBottomLimitSwitchSupplier() {
        return () -> getBottomLimitSwitch();
    }
    
    public void MoveUp() {
        if (!toplimitSwitch.get()) {
            Elevator.set(0);
        } else {
            Elevator.set(0.8);
        }
    }
    public void MoveDown() {
        if (!bottomlimitSwitch.get()) {
            Elevator.set(0);
        } else {
            Elevator.set(-0.8);
        }
    }
}
