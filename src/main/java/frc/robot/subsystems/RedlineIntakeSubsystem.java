package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class RedlineIntakeSubsystem extends SubsystemBase implements AutoCloseable{
    private final Spark redline; 
    
    public RedlineIntakeSubsystem() {
        this.redline = new Spark(IntakeConstants.kRedlinePort);
    }

    @Override
    public void periodic() {
        if (!redline.isAlive()) {
            DriverStation.reportError("Redline Motor is not alive", false);
        }
    }
    public void setRedline(double speed) {
        if (Math.abs(speed) > 1.0) {
            DriverStation.reportError("Redline motor setpoint out of range: " + speed, false);
        }
        redline.set(speed);
    }

    public void stopMotor() {
        redline.stopMotor();
    }

    public double getLastMotorSet() {
        return redline.get();
    }
    
    @Override
    public void close() {
        redline.close();
    }

}
