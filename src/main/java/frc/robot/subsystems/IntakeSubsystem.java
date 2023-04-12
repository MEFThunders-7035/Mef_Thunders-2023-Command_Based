package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    public static final Spark Redline = new Spark(IntakeConstants.kRedlinePort);
    public static final CANSparkMax Neo = new CANSparkMax(IntakeConstants.kCanNeoPort, CANSparkMax.MotorType.kBrushless);
    public IntakeSubsystem() {
        
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Neo Temp", Neo.getMotorTemperature());
    }
    public void setRedline(double speed) {
        Redline.set(speed);
    }
    public void setNeo(double speed) {
        Neo.set(speed);
    }
    public Double getNeoPos() {
        return Neo.getEncoder().getPosition();
    }
    public Double getNeoVel() {
        return Neo.getEncoder().getVelocity();
    }
    

}
