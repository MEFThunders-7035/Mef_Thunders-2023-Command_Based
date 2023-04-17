package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HorizontalElevatorConstants;

public class VerticalElevatorSubsystem extends SubsystemBase{

    public static final Spark Elevator = new Spark(HorizontalElevatorConstants.kHorizontalElevatorPort);
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);
    private boolean topLimitSwitch_temp;
    private boolean bottomLimitSwitch_temp;


    public VerticalElevatorSubsystem() {
        topLimitSwitch_temp = getTopLimitSwitch();
        bottomLimitSwitch_temp = getBottomLimitSwitch();
        SmartDashboard.putBoolean("Top Limit Switch", getTopLimitSwitch());
        SmartDashboard.putBoolean("Bottom Limit Switch", getBottomLimitSwitch());
        Elevator.setInverted(true);
    }
    
    @Override
    public void periodic() {
        if (topLimitSwitch_temp != getTopLimitSwitch()) {
            topLimitSwitch_temp = getTopLimitSwitch();
            SmartDashboard.putBoolean("Top Limit Switch", getTopLimitSwitch());
        }
        if (bottomLimitSwitch_temp != getBottomLimitSwitch()) {
            bottomLimitSwitch_temp = getBottomLimitSwitch();
            SmartDashboard.putBoolean("Bottom Limit Switch", getBottomLimitSwitch());
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
        if (getTopLimitSwitch()) {
            Elevator.set(0);
            throw new RuntimeException("Top limit switch triggered");
        } else {
            Elevator.set(0.8);
        }
    }
    
    public void MoveDown() {
        if (getBottomLimitSwitch()) {
            Elevator.set(0);
            throw new RuntimeException("Bottom limit switch triggered");
        } else {
            Elevator.set(-0.8);
        }
    }
}
