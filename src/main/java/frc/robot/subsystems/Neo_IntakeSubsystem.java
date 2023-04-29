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
        Neo.set(speed);
    }
    
    public double getNeoLastSet() {
        return Neo.get();
    }
    public REVLibError getNeoError() {
        return Neo.getLastError();
    }
    public double getNeoRampRate() {
        return Neo.getOpenLoopRampRate();
    }

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
    public Double getNeoPos() {
        return Neo.getEncoder().getPosition();
    }
    public Double getNeoVel() {
        return Neo.getEncoder().getVelocity();
    }
    public void setNeoPos(double pos) {
        Neo.getEncoder().setPosition(pos);
    }
    

}
