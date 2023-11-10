package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VerticalElevatorConstants;

public class VerticalElevatorSubsystem extends SubsystemBase implements AutoCloseable{

    private final Spark elevatorMotor;
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);
    private boolean topLimitSwitchTemp;
    private boolean bottomLimitSwitchTemp;


    public VerticalElevatorSubsystem() {
        this.elevatorMotor = new Spark(VerticalElevatorConstants.kVerticalElevatorPort);
        topLimitSwitchTemp = getTopLimitSwitch();
        bottomLimitSwitchTemp = getBottomLimitSwitch();
        SmartDashboard.putBoolean("Top Limit Switch", getTopLimitSwitch());
        SmartDashboard.putBoolean("Bottom Limit Switch", getBottomLimitSwitch());
        elevatorMotor.setInverted(true);
    }
    
    @Override
    public void close() throws Exception {
        elevatorMotor.close();
        toplimitSwitch.close();
        bottomlimitSwitch.close();
    }

    @Override
    public void periodic() {
        if (topLimitSwitchTemp != getTopLimitSwitch()) {
            topLimitSwitchTemp = getTopLimitSwitch();
            SmartDashboard.putBoolean("Top Limit Switch", getTopLimitSwitch());
        }
        if (bottomLimitSwitchTemp != getBottomLimitSwitch()) {
            bottomLimitSwitchTemp = getBottomLimitSwitch();
            SmartDashboard.putBoolean("Bottom Limit Switch", getBottomLimitSwitch());
        }

        if (!elevatorMotor.isAlive()) {
            DriverStation.reportError("Elevator motor is not alive", false);
        }
    }

    /**
     * Stops the elevator motor
     */
    public void stopMotor() {
        elevatorMotor.stopMotor();
    }

    /**
     * Sets the motor value
     * @param speed double between -1 and 1
     */
    public void setMotor(double speed) {
        elevatorMotor.set(speed);
    }

    /**
     * Gets the last set motor value
     * @return the last set motor value double between -1 and 1
     */
    public double getLastMotorSet() {
        return elevatorMotor.get();
    }

    public boolean getTopLimitSwitch() {
        return !toplimitSwitch.get();
    }

    public BooleanSupplier getTopLimitSwitchSupplier() {
        return this::getTopLimitSwitch;
    }
    
    public boolean getBottomLimitSwitch() {
        return !bottomlimitSwitch.get();
    }
    
    public BooleanSupplier getBottomLimitSwitchSupplier() {
        return this::getBottomLimitSwitch;
    }
    
    public void moveUp() {
        if (getTopLimitSwitch()) {
            elevatorMotor.set(0);
            DriverStation.reportWarning("Top limit switch triggered!", false);
        } else {
            elevatorMotor.set(0.8);
        }
    }
    
    public void moveDown() {
        if (getBottomLimitSwitch()) {
            elevatorMotor.set(0);
            DriverStation.reportWarning("Buttom limit switch triggered!", false);
        } else {
            elevatorMotor.set(-0.8);
        }
    }
}
