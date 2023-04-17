package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Redline_IntakeSubsystem extends SubsystemBase{
    public static final Spark Redline = new Spark(IntakeConstants.kRedlinePort);
    public Redline_IntakeSubsystem() {
        
    }

    public void periodic() {
        if (!Redline.isAlive()) {
            DriverStation.reportError("Redline Motor is not alive", false);
            throw new RuntimeException("Redline motor is not alive");
        }
    }
    public void setRedline(double speed) {
        Redline.set(speed);
    }

}
