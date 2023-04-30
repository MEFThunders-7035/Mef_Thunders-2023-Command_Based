package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VerticalElevatorConstants;


public class HorizontalElevatorSubsystem extends SubsystemBase implements AutoCloseable{

    private final Spark Vertical_Elevator;
    
    public HorizontalElevatorSubsystem() {
        this.Vertical_Elevator = new Spark(VerticalElevatorConstants.kVerticalElevatorPort);
        Vertical_Elevator.setInverted(true);
    }

    @Override
    public void close() throws Exception {
        Vertical_Elevator.close();
    }

    @Override
    public void periodic() {
        if (!Vertical_Elevator.isAlive()) {
            DriverStation.reportError("Vertical Elevator motor is not alive", false);
            throw new RuntimeException("Vertical Elevator motor is not alive");
        }
    }

    /**
     * Stops the elevator motor
     */
    public void stop() {
        Vertical_Elevator.stopMotor();
    }

    /**
     * Sets the motor value
     * @param speed double between -1 and 1
     */
    public void setMotor(double speed) {
        if (Math.abs(speed) > 1) {
            DriverStation.reportError("Horizontal Elevator motor speed is out of range", false);
            throw new IllegalArgumentException("Horizontal elevator motor setpoint out of range: " + speed);
        }
        Vertical_Elevator.set(speed);
    }
    
    /**
     * Gets the last set motor value
     * @return the last set motor value double between -1 and 1
     */
    public double getLastMotorSet() {
        return Vertical_Elevator.get();
    }
}
