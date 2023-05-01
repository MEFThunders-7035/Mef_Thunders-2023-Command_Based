package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Neo_IntakeSubsystem extends SubsystemBase implements AutoCloseable{

    public static final CANSparkMax Neo = new CANSparkMax(IntakeConstants.kCanNeoPort, CANSparkMax.MotorType.kBrushless);
    
    public Neo_IntakeSubsystem() {
        
    }

    @Override
    public void close() throws Exception {
        return;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Neo Encoder", Neo.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo Velocity", Neo.getEncoder().getVelocity());
        SmartDashboard.putNumber("Neo Temp", Neo.getMotorTemperature());
    }
    public void setNeo(double speed) {
        if (Math.abs(speed) > 1) {
            throw new IllegalArgumentException("Neo setpoint out of range: " + speed);
        }
        Neo.set(speed);
    }
    
    /**
     * gets the last set motor value
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double getNeoLastSet() {
        return Neo.get();
    }

    /**
     * Get the last error of the Neo
     * @return the last error of the Neo as a  {@link REVLibError}
     */
    public REVLibError getNeoError() {
        return Neo.getLastError();
    }

    /**
     * Get the configured open loop ramp rate.
     * This is the maximum rate at which the motor controller's output is allowed to change.
     * @return ramp rate time in seconds to go from 0 to full throttle.
     */
    public double getNeoRampRate() {
        return Neo.getOpenLoopRampRate();
    }

    /**
     * Get the idle mode of the Neo
     * @return true = brake, false = coast
     */
    public boolean getNeoBrakeMode() {
        if (Neo.getIdleMode().equals(CANSparkMax.IdleMode.kBrake)) {
            return true;
        } else {
            return false;
        }
    }
    
    /**
     * get the fault bits of the Neo
     * @return the fault bits of the Neo as a short
     */
    public short getNeoFaults() {
        return Neo.getFaults();
    }

    /**
     * Set the ramp rate of the Neo
     * @param rate the ramp rate in seconds from 0 to full speed
     */
    public void setNeoRampRate(double rate) {
        Neo.setOpenLoopRampRate(rate);
    }

    public void setNeoCurrentLimit(int limit) {
        Neo.setSmartCurrentLimit(limit);
    }

    
    /**
     * Stop the Neo
     */
    public void stopNeo() {
        Neo.stopMotor();
    }

    /**
     * Set the idle mode of the Neo
     * @param brake true = brake, false = coast
     */
    public void setNeoBrake(boolean brake) {
        Neo.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
    /**
     * @return the motor temperature of the Neo in Celsius
     */
    public double getNeoTempreture() {
        return Neo.getMotorTemperature();
    }

    /**
     * returns the Neo's encoder position
     * @return the Neo's encoder position in rotations (1 rotation = 1 full rotation of the motor)
     */
    public Double getNeoPos() {
        return Neo.getEncoder().getPosition();
    }

    /**
     * returns the Neo's encoder velocity
     * @return the Neo's encoder velocity in RPM
     */
    public Double getNeoVel() {
        return Neo.getEncoder().getVelocity();
    }

    /**
     * sets the Neo's encoder position
     * @param pos the position to set the Neo's encoder to in rotations (1 rotation = 1 full rotation of the motor)
     */
    public void setNeoPos(double pos) {
        Neo.getEncoder().setPosition(pos);
    }
    

}
