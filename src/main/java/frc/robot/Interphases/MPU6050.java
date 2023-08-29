package frc.robot.Interphases;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;


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
    private double Y_angle_offset;

    private double rate_offset;
    private double X_rate_offset;
    private double Y_rate_offset;

    private double X_Accel_offset;
    private double Y_Accel_offset;
    private double Z_Accel_offset;
    
    private double angleX;
    private double angleY;
    private double angleZ;
    
    private double LoopTime;
    private double currentTimestamp;
    private double lastTimestamp;

    LinearFilter Xfilter;
    LinearFilter Yfilter;
    LinearFilter Zfilter;


    /**
     * Creates a new instance of the MPU6050 class.
     * @param port The I2C port to which the sensor is connected.
     */
    public MPU6050(I2C.Port port) {
        mpu6050 = new I2C(port, DEVICE_ADDRESS);
        mpu6050.write(PWR_MGMT_1, 0); // Wake up the sensor
        LoopTime = 0.2;
        angle_offset = 0;
        rate_offset = 0;
        X_rate_offset = 0;
        Y_rate_offset = 0;
        X_Accel_offset = 0;
        X_Accel_offset = 0;
        Z_Accel_offset = 0;
        angleX = 0;
        angleY = 0;
        angleZ = 0;

        Xfilter = LinearFilter.singlePoleIIR(0.1, LoopTime);
        Yfilter = LinearFilter.singlePoleIIR(0.1, LoopTime);
        Zfilter = LinearFilter.singlePoleIIR(0.1, LoopTime);
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
     * Runs all the calculations to get the angle data, so it's important to run this periodically.
     * @apiNote RUN IT PERIODICALLY. 
     */
    public void update() {
        currentTimestamp = Timer.getFPGATimestamp();
        LoopTime = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;
        
        // Assuming X axis pointing forward, the Y axis pointing left, and the Z axis pointing up. (probably not the case here)
                
        double rateX = this.getRateX();
        double rateY = this.getRateY();
        double rateZ = this.getRate();

        double accelX = this.getAccelX();
        double accelY = this.getAccelY();
        double accelZ = this.getAccelZ();

        double accelAngleX = Math.atan2(accelY, accelZ) * 180.0 / Math.PI;
        double accelAngleY = Math.atan2(-accelX, Math.sqrt(accelY * accelY + accelZ * accelZ)) * 180 / Math.PI;

        //angleX = DriveConstants.kGyroFilterStrenght * (angleX + (rateX * LoopTime)) + (1 - DriveConstants.kGyroFilterStrenght) * angleAccelX;
        //angleY = DriveConstants.kGyroFilterStrenght * (angleY + (rateY * LoopTime)) + (1 - DriveConstants.kGyroFilterStrenght) * angleAccelY;

        rateX = Xfilter.calculate(rateX);
        rateY = Yfilter.calculate(rateY);
        rateZ = Zfilter.calculate(rateZ);
        
        angleX += rateX * LoopTime;
        angleY += rateY * LoopTime;
        angleZ += rateZ * LoopTime;
    }
    

    /**
     * Calibrate the gyro. 
     * <p>It's important to make sure that the robot is not moving while the calibration is in progress, 
     * this is typically done when the robot is first turned on while it's sitting at rest before the match starts.<p>
     * 
     * @apiNote The calibration process takes approximately 5 seconds to complete. And is done on another thread
     */
    @Override
    public void calibrate() {
        System.out.println("Starting Calibration");
        AccCalibrate();
        //for some reason taking the highest value gives the best results.
        if (rate_offset != 0) {
            System.out.println("Rate Offset is already not 0");
            return;
        }

        new Thread(() -> {
            double Xoffset = 0;
            double Yoffset = 0;
            double Zoffset = 0;
            double XaccelOffset = 0.0;
            double YaccelOffset = 0.0;
            double ZaccelOffset = 0.0;
            for (int i = 0; i < 500; i++) {
            Zoffset += getRate();
            Xoffset += getRateX();
            Yoffset += getRateY();
            XaccelOffset += getAccelX();
            YaccelOffset += getAccelY();
            ZaccelOffset += getAccelZ();
            Timer.delay(0.01);
            }
            rate_offset = Zoffset / 500;
            X_rate_offset = Xoffset / 500;
            Y_rate_offset = Yoffset / 500;
            X_Accel_offset = XaccelOffset / 500;
            Y_Accel_offset = YaccelOffset / 500;
            Z_Accel_offset = ZaccelOffset / 500;
            System.out.println("Calibration Complete! Rate_Offstet: " + rate_offset);
        }).start();
    }

    private void AccCalibrate() {
        X_Accel_offset = getAccelX();
        Y_Accel_offset = getAccelY();
        Z_Accel_offset = getAccelZ();
    }

    @Override
    public void reset() {
        angle_offset = angleZ;
        X_angle_offset = angleX;
        Y_angle_offset = angleY;
    }

    /**
     * Gets the angle of the sensor.
     * @return The angle of the sensor in degrees does not go higher than 360 and resets back to zero.
     * @apiNote use {@link #getAngle()} if you want it to be continous.
     */
    public double getGyroAngleFixed() {
        return getAngle() % 360;
    }

    @Override
    public double getAngle() {
        /*
        double rate = getRate();
        angleZ += rate * LoopTime;
        double gyro_angle = angleZ - angle_offset;
        double acc_angle =  Units.radiansToDegrees(Math.atan2(getAccelY(), getAccelZ()));
        SmartDashboard.putNumber("Acc Angle", acc_angle);
        */
        return angleZ - angle_offset;
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
        return angleX - X_angle_offset;
    }

    public double getAngleY() {
        return angleY;
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
    public double getRateX() {
        return (readShort(GYRO_XOUT_H) / 131.0) - X_rate_offset;
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getRateY() {
        return ((readShort(GYRO_YOUT_H)) / 131.0) - Y_rate_offset;
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getRateZ() {
        byte[] buffer = new byte[6];
        mpu6050.read(GYRO_ZOUT_H, 6, buffer);
        int z = (buffer[4] << 8) | buffer[5];
        return ((double) z / 131.0) - rate_offset;
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
