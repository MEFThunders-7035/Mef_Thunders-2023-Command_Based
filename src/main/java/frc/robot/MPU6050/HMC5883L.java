package frc.robot.MPU6050;

import edu.wpi.first.wpilibj.I2C;

public class HMC5883L{
    
    private static final byte DEVICE_ADDRESS = 0x1E;

    private static final byte REGISTER_CONFIG_A = 0x00;
    private static final byte REGISTER_CONFIG_B = 0x01;
    private static final byte REGISTER_MODE = 0x02;
    
    
    private final I2C hmc5883L;

    public HMC5883L(I2C.Port port) {
        hmc5883L = new I2C(port, DEVICE_ADDRESS);
    }

    public void init() {

    }

    private byte[] read(int register, int count) {
        byte[] buffer = new byte[count];
        hmc5883L.read(register, count, buffer);
        return buffer;
    }

    private short readShort(int register) {
        return 0;
    }
}
