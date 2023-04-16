package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Neo_IntakeSubsystem extends SubsystemBase{

    public static final CANSparkMax Neo = new CANSparkMax(IntakeConstants.kCanNeoPort, CANSparkMax.MotorType.kBrushless);
    public Neo_IntakeSubsystem() {
        
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
