package frc.robot.Interphases;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MPU6050 implements Gyro{
    private static final int DEVICE_ADDRESS = 0x68;
    private static final int PWR_MGMT_1 = 0x6B;
    private static final int GYRO_XOUT_H = 0x43;
    private static final int GYRO_YOUT_H = 0x45;
    private static final int GYRO_ZOUT_H = 0x47;
    private static final int ACCEL_XOUT_H = 0x3B;
    private static final int ACCEL_YOUT_H = 0x3D;
    private static final int ACCEL_ZOUT_H = 0x3F;
    
    private final I2C mpu6050;
    
    private double angle_offset;
    private double X_angle_offset;
    private double rate_offset;
    private double X_rate_offset;
    private double X_Accel_offset;
    private double Y_Accel_offset;
    private double Z_Accel_offset;
    private double angle;
    private double X_angle;
    private double LoopTime;

    /**
     * Creates a new instance of the MPU6050 class.
     * @param port The I2C port to which the sensor is connected.
     */
    public MPU6050(I2C.Port port) {
        mpu6050 = new I2C(port, DEVICE_ADDRESS);
        mpu6050.write(PWR_MGMT_1, 0);
        LoopTime = 0.2;
        angle_offset = 0;
        rate_offset = 0;
        X_rate_offset = 0;
        X_Accel_offset = 0;
        X_Accel_offset = 0;
        Z_Accel_offset = 0;
        angle = 0;
        X_angle = 0;
        // write(0x1B, (byte) 0x08); // Set full scale range for gyro
        // write(0x1C, (byte) 0x08); // Set full scale range for accelerometer
    }

    /**
     * Reads the specified number of bytes from the specified register on the sensor.
     * @param register The register to read from.
     * @param count The number of bytes to read.
     * @return The bytes read from the sensor.
     */
    private byte[] read(int register, int count) {
        byte[] buffer = new byte[count];
        mpu6050.read(register, count, buffer);
        return buffer;
    }

    /**
     * Reads the specified register on the sensor.
     * @param register The register to read.
     * @return The value read from the sensor.
     */
    private short readShort(int register) {
        byte[] buffer = read(register, 2);
        return (short) ((buffer[0] << 8) | buffer[1]);
    }

    /**
     * Closes the connection to the sensor.
     */
    @Override
    public void close() throws Exception {
        mpu6050.close();
    }

    /**
     * Calibrate the gyro. 
     * <p>It's important to make sure that the robot is not moving while the calibration is in progress, 
     * this is typically done when the robot is first turned on while it's sitting at rest before the match starts.<p>
     * 
     * @apiNote The calibration process takes 5 seconds to complete.
     */
    @Override
    public void calibrate() {
        System.out.println("Starting Calibration");
        angle_offset = getAngle();
        X_angle_offset = getAngleX();
        AccCalibrate();
        //for some reason taking the highest value gives the best results.
        for (int i = 0; i < 200; i++) {
            if (Math.abs(X_rate_offset) < Math.abs(getGyro_Rate_X())) {
                X_rate_offset = getGyro_Rate_X();
            }

            if (Math.abs(rate_offset) < Math.abs(getRate())) {
                rate_offset = getRate();
            }
            try {
                Thread.sleep(25);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        System.out.println("Calibration Complete! Rate_Offstet: " + rate_offset);
    }

    public void AccCalibrate() {
        X_Accel_offset = getAccelX();
        Y_Accel_offset = getAccelY();
        Z_Accel_offset = getAccelZ();
    }

    @Override
    public void reset() {
        angle = 0;
    }

    /**
     * Gets the angle of the sensor.
     * @return The angle of the sensor in degrees does not go higher than 360 and resets back to zero.
     * @apiNote use {@link #getAngle()} if you want it to be continous.
     */
    public double getGyroAngleFixed() {
        return getAngle() % 360;
    }

    /**
     * Sets the loop time for the gyro, deafult is 0.2 seconds.
     * @param period The loop time in seconds.
     * @apiNote This is used to calculate the angle of the gyro So RUN IT PERIODICALLY. 
     */
    public void setLoopTime(double period) {
        LoopTime = period;
    }

    @Override
    public double getAngle() {
        double rate = getRate();
        angle += rate * LoopTime;
        double gyro_angle = angle - angle_offset;
        double acc_angle =  Units.radiansToDegrees(Math.atan2(getAccelY(), getAccelZ()));
        SmartDashboard.putNumber("Acc Angle", acc_angle);
        return gyro_angle;
    }

    /**
     * Return The heading of the X axis.
     * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. 
     * This allows algorithms that wouldn't want to see a discontinuity in the gyro output 
     * as it sweeps past from 360 to 0 on the second time around.<p>
     * <p> This heading is based on integration of the returned rate from the gyro. <p>
     * @return The current X angle of the robot in degrees.
     */
    public double getAngleX() {
        double rate = getGyro_Rate_X();
        X_angle += rate * LoopTime;
        return X_angle - X_angle_offset;
    }

    @Override
    public double getRate() {
        byte[] buffer = new byte[6];
        mpu6050.read(GYRO_XOUT_H, 6, buffer);
        int z = (buffer[4] << 8) | buffer[5];
        return ((double) z / 131.0) - rate_offset;
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getGyro_Rate_X() {
        return readShort(GYRO_XOUT_H) / 131.0;
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getGyro_Rate_Y() {
        return (readShort(GYRO_YOUT_H) - X_rate_offset) / 131.0;
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getGyro_Rate_Z() {
        return (readShort(GYRO_ZOUT_H) - rate_offset) / 131.0;
    }

    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelX() {
        return (readShort(ACCEL_XOUT_H) - X_Accel_offset) / 16384.0;
    }

    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelY() {
        return (readShort(ACCEL_YOUT_H) - Y_Accel_offset) / 16384.0;
    }

    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelZ() {
        return (readShort(ACCEL_ZOUT_H) - Z_Accel_offset) / 16384.0;
    }
    
    /**
     * Sets the offset of the sensor. 
     * @apiNote IS DONE AUTOMATICALLY IN {@link #calibrate()}.
     * @param offset The offset to set in degrees.
     */
    public void setAngle_offset(double offset) {
        this.angle_offset = offset;
    }

    /**
     * @return The current offset of the sensor in degrees.
     */
    public double getAngle_offset() {
        return angle_offset;
    }

    public double getRate_offset() {
        return rate_offset;
    }
}
