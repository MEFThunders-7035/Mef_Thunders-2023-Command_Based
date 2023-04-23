package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
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
    
    /**
     * Uses the Accelerometer to find the position and velocity of the robot and updates the Field2d, warning this is not very accurate and is not recommended for use. 
     * @apiNote Only use this if you have no other way of finding the position of the robot.
     */
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

        positionX = field.getRobotPose().getTranslation().getX();
        positionY = field.getRobotPose().getTranslation().getY();

        double dX = velocityX * dt;
        double dY = velocityY * dt;
        double dZ = velocityZ * dt;

        Translation2d rotatedTranslation = new Translation2d(dX, dY).rotateBy(field.getRobotPose().getRotation());
        
        positionX += rotatedTranslation.getX();
        positionY += rotatedTranslation.getY();
        positionZ += dZ;

        field.setRobotPose(positionX, positionY, field.getRobotPose().getRotation());
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("PositionX", positionX);
        SmartDashboard.putNumber("PositionY", positionY);
        SmartDashboard.putNumber("PositionZ", positionZ);
        SmartDashboard.putNumber("VelocityX", velocityX);
        SmartDashboard.putNumber("VelocityY", velocityY);
        SmartDashboard.putNumber("VelocityZ", velocityZ);
    }

    /**
     * Resets the position and velocity of the robot
     */
    public void resetAll() {
        positionX = 0;
        positionY = 0;
        positionZ = 0;
        velocityX = 0;
        velocityY = 0;
        velocityZ = 0;
    }
    
    public void resetPosition() {
        positionX = 0;
        positionY = 0;
        positionZ = 0;
    }

    public void resetVelocity() {
        velocityX = 0;
        velocityY = 0;
        velocityZ = 0;
    }

    /**
     * Gets the Velocity of the robot as an array using the integration of the acceleration
     * @return array of doubles with the velocity of the robot {x, y, z}
     */    
    public double[] getVelocityArray() {
        return new double[] {velocityX, velocityY, velocityZ};
    }
    
    /**
     * Gets the Position of the robot as an array using double integration of the acceleration
     * @return array of doubles with the position of the robot {x, y, z}
     */
    public double[] getPositionArray() {
        return new double[] {positionX, positionY, positionZ};
    }

    /**
     * Finds the Position of the robot trough double integration of the acceleration
     * @return x position of the robot in double
     */
    public double getPositionX() {
        return positionX;
    }

    /**
     * Finds the Position of the robot trough double integration of the acceleration
     * @return y position of the robot in double
     */
    public double getPositionY() {
        return positionY;
    }

    /**
     * Finds the Position of the robot trough double integration of the acceleration
     * @return z position of the robot in double
     */
    public double getPositionZ() {
        return positionZ;
    }

    /**
     * Finds the Velocity of the robot trough integration of the acceleration
     * @return x velocity of the robot in double
     */
    public double getVelocityX() {
        return velocityX;
    }

    /**
     * Finds the Velocity of the robot trough integration of the acceleration
     * @return y velocity of the robot in double
     */
    public double getVelocityY() {
        return velocityY;
    }

    /**
     * Finds the Velocity of the robot trough integration of the acceleration
     * @return z velocity of the robot in double
     */
    public double getVelocityZ() {
        return velocityZ;
    }

    /**
     * Finds the Acceleration of the robot trough the accelerometer
     * @return x acceleration of the robot in double
     */
    public double getAccelerationX() {
        return accel.getX();
    }

    /**
     * Finds the Acceleration of the robot trough the accelerometer
     * @return y acceleration of the robot in double
     */
    public double getAccelerationY() {
        return accel.getY();
    }

    /**
     * Finds the Acceleration of the robot trough the accelerometer
     * @return z acceleration of the robot in double
     */
    public double getAccelerationZ() {
        return accel.getZ();
    }

    /**
     * Sets the initial position of the robot using an array
     * @param initialVelocity array of doubles with the initial velocity of the robot {x, y, z}
     * @throws IllegalArgumentException if the array is not of length 3
     */
    public void setInitialVelocity(double[] initialVelocity) {
        if (initialVelocity.length != 3) {
            throw new IllegalArgumentException("Array must be of length 3");
        }
        velocityX = initialVelocity[0];
        velocityY = initialVelocity[1];
        velocityZ = initialVelocity[2];
    }

    /**
     * Sets the initial position of the robot using an array
     * @param initialVelocityX initial velocity of the robot in the x direction
     * @param initialVelocityY initial velocity of the robot in the y direction
     * @param initialVelocityZ initial velocity of the robot in the z direction
     */
    public void setInitialVelocity(double initialVelocityX, double initialVelocityY, double initialVelocityZ) {
        velocityX = initialVelocityX;
        velocityY = initialVelocityY;
        velocityZ = initialVelocityZ;
    }

    /**
     * Sets the initial position of the robot using an array
     * @param initialPosition array of doubles with the initial position of the robot {x, y, z}
     * @throws IllegalArgumentException if the array is not of length 3
     */
    public void setInitialPosition(double[] initialPosition) {
        if (initialPosition.length != 3) {
            throw new IllegalArgumentException("Array must be of length 3");
        }
        positionX = initialPosition[0];
        positionY = initialPosition[1];
        positionZ = initialPosition[2];
    }

    /**
     * Sets the initial position of the robot using an array
     * @param initialPositionX initial position of the robot in the x direction
     * @param initialPositionY initial position of the robot in the y direction
     * @param initialPositionZ initial position of the robot in the z direction
     */
    public void setInitialPosition(double initialPositionX, double initialPositionY, double initialPositionZ) {
        positionX = initialPositionX;
        positionY = initialPositionY;
        positionZ = initialPositionZ;
      }

}
