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

    public MPU6050(I2C.Port port) {
        mpu6050 = new I2C(port, DEVICE_ADDRESS);
        mpu6050.write(PWR_MGMT_1, 0);
        LoopTime = 0.2;
        offset = 0;
        angle = 0;
        write(0x1B, (byte) 0x08); // Set full scale range for gyro
        write(0x1C, (byte) 0x08); // Set full scale range for accelerometer
    }

    public void write(int register, byte value) {
        mpu6050.write(register, value);
    }

    public byte[] read(int register, int count) {
        byte[] buffer = new byte[count];
        mpu6050.read(register, count, buffer);
        return buffer;
    }

    public short readShort(int register) {
        byte[] buffer = read(register, 2);
        return (short) ((buffer[0] << 8) | buffer[1]);
    }

    public double getGyroX() {
        return readShort(GYRO_XOUT_H) / 131.0;
    }

    public double getGyroY() {
        return readShort(GYRO_YOUT_H) / 131.0;
    }

    public double getGyroZ() {
        return readShort(GYRO_ZOUT_H) / 131.0;
    }

    public double getAccelX() {
        return readShort(ACCEL_XOUT_H) / 16384.0;
    }

    public double getAccelY() {
        return readShort(ACCEL_YOUT_H) / 16384.0;
    }

    public double getAccelZ() {
        return readShort(ACCEL_ZOUT_H) / 16384.0;
    }

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

    public double getGyroAngleFixed() {
        return getAngle() % 360;
    }

    /**
     * Sets the loop time for the gyro, will 
     * @param period
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

    public void addOffset(double offset) {
        this.offset += offset;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getOffset() {
        return offset;
    }
}
