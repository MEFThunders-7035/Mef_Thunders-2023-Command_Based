package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AcceleratorSubsystem extends SubsystemBase{
    
    private final Field2d field;
    Accelerometer accel = new BuiltInAccelerometer();
    private double lastTimestamp;
    private double velocityX = 0;
    private double velocityY = 0;
    private double velocityZ = 0;
    private double positionX = 0;
    private double positionY = 0;
    private double positionZ = 0;
    
    public AcceleratorSubsystem(Field2d field) {
        this.field = field;
    }

    @Override
    public void periodic() {
        double currentTimestamp = Timer.getFPGATimestamp() / 10;
        double dt = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;

        velocityX += getAccelerationX() * dt;
        velocityY += getAccelerationY() * dt;
        velocityZ += getAccelerationZ() * dt;

        positionX += velocityX * dt;
        positionY += velocityY * dt;
        positionZ += velocityZ * dt;

        field.setRobotPose(positionX, positionY, new Rotation2d());
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("PositionX", positionX);
        SmartDashboard.putNumber("PositionY", positionY);
        SmartDashboard.putNumber("PositionZ", positionZ);
        SmartDashboard.putNumber("VelocityX", velocityX);
        SmartDashboard.putNumber("VelocityY", velocityY);
        SmartDashboard.putNumber("VelocityZ", velocityZ);
    }

    public double[] getVelocityArray() {
        return new double[] {velocityX, velocityY, velocityZ};
    }
    
    public double[] getPositionArray() {
        return new double[] {positionX, positionY, positionZ};
    }

    public double getPositionX() {
        return positionX;
    }

    public double getPositionY() {
        return positionY;
    }

    public double getPositionZ() {
        return positionZ;
    }

    public double getVelocityX() {
        return velocityX;
    }

    public double getVelocityY() {
        return velocityY;
    }

    public double getVelocityZ() {
        return velocityZ;
    }

    public double getAccelerationX() {
        return accel.getX();
    }

    public double getAccelerationY() {
        return accel.getY();
    }

    public double getAccelerationZ() {
        return accel.getZ();
    }

    public void setInitialVelocity(double initialVelocityX, double initialVelocityY, double initialVelocityZ) {
        velocityX = initialVelocityX;
        velocityY = initialVelocityY;
        velocityZ = initialVelocityZ;
    }

    public void setInitialPosition(double initialPositionX, double initialPositionY, double initialPositionZ) {
        positionX = initialPositionX;
        positionY = initialPositionY;
        positionZ = initialPositionZ;
      }

}
