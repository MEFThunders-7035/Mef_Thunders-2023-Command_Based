package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Redline_IntakeSubsystem extends SubsystemBase implements AutoCloseable{
    private final Spark Redline; 
    
    public Redline_IntakeSubsystem() {
        this.Redline = new Spark(IntakeConstants.kRedlinePort);
    }

    public void periodic() {
        if (!Redline.isAlive()) {
            DriverStation.reportError("Redline Motor is not alive", false);
            throw new RuntimeException("Redline motor is not alive");
        }
    }
    public void setRedline(double speed) {
        if (Math.abs(speed) > 1.0) {
            DriverStation.reportError("Redline motor setpoint out of range: " + speed, false);
            throw new IllegalArgumentException("Redline motor setpoint out of range: " + speed);
        }
        Redline.set(speed);
    }

    public void stopMotor() {
        Redline.stopMotor();
    }

    public double getLastMotorSet() {
        return Redline.get();
    }
    
    @Override
    public void close() {
        Redline.close();
    }

}
