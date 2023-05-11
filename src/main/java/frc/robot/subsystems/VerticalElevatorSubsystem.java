package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VerticalElevatorConstants;

public class VerticalElevatorSubsystem extends SubsystemBase implements AutoCloseable{

    private final Spark Elevator;
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);
    private boolean topLimitSwitch_temp;
    private boolean bottomLimitSwitch_temp;


    public VerticalElevatorSubsystem() {
        this.Elevator = new Spark(VerticalElevatorConstants.kVerticalElevatorPort);
        topLimitSwitch_temp = getTopLimitSwitch();
        bottomLimitSwitch_temp = getBottomLimitSwitch();
        SmartDashboard.putBoolean("Top Limit Switch", getTopLimitSwitch());
        SmartDashboard.putBoolean("Bottom Limit Switch", getBottomLimitSwitch());
        Elevator.setInverted(true);
    }
    
    @Override
    public void close() throws Exception {
        Elevator.close();
        toplimitSwitch.close();
        bottomlimitSwitch.close();
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

    /**
     * Stops the elevator motor
     */
    public void stop() {
        Elevator.stopMotor();
    }

    /**
     * Sets the motor value
     * @param speed double between -1 and 1
     */
    public void setMotor(double speed) {
        if (Math.abs(speed) > 1) {
            DriverStation.reportError("Elevator motor speed is out of range", false);
            throw new IllegalArgumentException("Vertical elevator motor setpoint out of range: " + speed);
        }
        Elevator.set(speed);
    }

    /**
     * Gets the last set motor value
     * @return the last set motor value double between -1 and 1
     */
    public double getLastMotorSet() {
        return Elevator.get();
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
