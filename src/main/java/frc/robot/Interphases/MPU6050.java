package frc.robot.Interphases;

import edu.wpi.first.wpilibj.I2C;
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
    private double offset;
    private double angle;
    private double LoopTime;

    /**
     * Creates a new instance of the MPU6050 class.
     * @param port The I2C port to which the sensor is connected.
     */
    public MPU6050(I2C.Port port) {
        mpu6050 = new I2C(port, DEVICE_ADDRESS);
        mpu6050.write(PWR_MGMT_1, 0);
        LoopTime = 0.2;
        offset = 0;
        angle = 0;
        // write(0x1B, (byte) 0x08); // Set full scale range for gyro
        // write(0x1C, (byte) 0x08); // Set full scale range for accelerometer
    }

    /**
     * Writes a byte value to the specified register on the sensor.
     * @param register The register to write to.
     * @param value The value to write.
     */
    public void write(int register, byte value) {
        mpu6050.write(register, value);
    }

    /**
     * Reads the specified number of bytes from the specified register on the sensor.
     * @param register The register to read from.
     * @param count The number of bytes to read.
     * @return The bytes read from the sensor.
     */
    public byte[] read(int register, int count) {
        byte[] buffer = new byte[count];
        mpu6050.read(register, count, buffer);
        return buffer;
    }

    /**
     * Reads the specified register on the sensor.
     * @param register The register to read.
     * @return The value read from the sensor.
     */
    public short readShort(int register) {
        byte[] buffer = read(register, 2);
        return (short) ((buffer[0] << 8) | buffer[1]);
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getGyroX() {
        return readShort(GYRO_XOUT_H) / 131.0;
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getGyroY() {
        return readShort(GYRO_YOUT_H) / 131.0;
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getGyroZ() {
        return readShort(GYRO_ZOUT_H) / 131.0;
    }

    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelX() {
        return readShort(ACCEL_XOUT_H) / 16384.0;
    }

    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelY() {
        return readShort(ACCEL_YOUT_H) / 16384.0;
    }

    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelZ() {
        return readShort(ACCEL_ZOUT_H) / 16384.0;
    }
    
    /**
     * Closes the connection to the sensor.
     */
    @Override
    public void close() throws Exception {
        mpu6050.close();
    }

    @Override
    public void calibrate() {
        offset = getAngle();
    }

    @Override
    public void reset() {
        angle = 0;
    }

    /**
     * Gets the angle of the sensor.
     * @return The angle of the sensor in degrees does not go higher than 360 and resets back to zero.
     * @see #getAngle() if you want it to be continous.
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
        return angle - offset;
    }

    @Override
    public double getRate() {
        byte[] buffer = new byte[6];
        mpu6050.read(GYRO_XOUT_H, 6, buffer);
        int x = (buffer[0] << 8) | buffer[1];
        int y = (buffer[2] << 8) | buffer[3];
        int z = (buffer[4] << 8) | buffer[5];
        return (double) z / 131.0;
    }

    /**
     * lets you add a spesific offset to the sensor.
     * WARNING DOES NOT SET THE OFFSET IT ADDS TO THE CURRENT OFFSET
     * See {@link #setOffset(double)} if you want to set the offset.
     * @param offset
     */
    public void addOffset(double offset) {
        this.offset += offset;
    }

    /**
     * Sets the offset of the sensor. 
     * @apiNote IS DONE AUTOMATICALLY IN {@link #calibrate()}.
     * @param offset The offset to set in degrees.
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    /**
     * @return The current offset of the sensor in degrees.
     */
    public double getOffset() {
        return offset;
    }
}