package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase implements AutoCloseable{

    public static final CANSparkMax IntakeArmMotor = new CANSparkMax(IntakeConstants.kCanIntakeArmMotor1Port, CANSparkMax.MotorType.kBrushed);
    public static final Spark IntakeArmMotor2 = new Spark(IntakeConstants.kCanIntakeArmMotor2Port);
    public static final AnalogPotentiometer IntakeArmPot = new AnalogPotentiometer(IntakeConstants.kIntakeArmPotPort);
    
    public IntakeArmSubsystem() {
        IntakeArmMotor.setInverted(true);
    }

    @Override
    public void close() throws Exception {
        return;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Potantiometer", getArmPos());
    }
    public void setMotors(double speed) {
        if (Math.abs(speed) > 1) {
            DriverStation.reportWarning("Motors setpoint out of range: " + speed, false);
        }
        IntakeArmMotor.set(speed);
        IntakeArmMotor2.set(speed);
    }
    
    /**
     * gets the last set motor value
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double getMotorsLastSet() {
        return IntakeArmMotor.get();
    }

    /**
     * Get the idle mode of the Motors
     * @return true = brake, false = coast
     */
    public boolean getMotorsBrakeMode() {
        if (IntakeArmMotor.getIdleMode().equals(CANSparkMax.IdleMode.kBrake)) {
            return true;
        } else {
            return false;
        }
    }
    
    /**
     * Stop the Motors
     */
    public void stopMotors() {
        IntakeArmMotor.stopMotor();
    }

    /**
     * Set the idle mode of the Motors
     * @param brake true = brake, false = coast
     */
    public void setMotorsBrake(boolean brake) {
        IntakeArmMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
    /**
     * Uses the potentiometer to get the position of the arm.
     * @return the position of the arm in degrees
     */
    public double getArmPos() {
        return IntakeArmPot.get();
    }
}
