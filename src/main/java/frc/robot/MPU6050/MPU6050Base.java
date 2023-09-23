package frc.robot.MPU6050;

import static frc.robot.MPU6050.MPU6050Constants.AutoGenerated.MPU6050_DEFAULT_ADDRESS;
import static frc.robot.MPU6050.MPU6050Constants.AutoGenerated.MPU6050_DMP_MEMORY_BANK_SIZE;
import static frc.robot.MPU6050.MPU6050Constants.AutoGenerated.MPU6050_RA_BANK_SEL;
import static frc.robot.MPU6050.MPU6050Constants.AutoGenerated.MPU6050_RA_MEM_R_W;

import edu.wpi.first.wpilibj.I2C;


public abstract class MPU6050Base{
    BetterI2C mpu6050;
    
    public MPU6050Base(I2C.Port port) {
        this(port, MPU6050_DEFAULT_ADDRESS);
    }
    
    public MPU6050Base(I2C.Port port, int address) {
        mpu6050 = new BetterI2C(port, address);
        initialize();
    }

    long map(long x, long in_min, long in_max, long out_min, long out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Write to the DMP memory.
     * This function prevents I2C writes past the bank boundaries. The DMP memory
     * is only accessible when the chip is awake.
     * @param mem_addr Memory location (bank << 8 | start address)
     * @param length Number of bytes to write.
     * @param data Bytes to write to memory.
     * @return Transfer Aborted... false for success, true for aborted.
    */
    protected boolean writeMem(short mem_addr, short length, char[] data) {
        byte[] tmp = new byte[2];
        byte[] tmp2 = new byte[data.length];
        
        for (int i = 0; i < data.length; i++) {
            tmp2[i] = (byte) data[i];
        }

        tmp[0] = (byte) (mem_addr >> 8);
        tmp[1] = (byte) (mem_addr & 0xFF);

        // Check bank boundaries
        if (tmp[1] + length > MPU6050_DMP_MEMORY_BANK_SIZE) {
            return true;
        }
        
        // After this point, I couldn't find any documentation on what this code does, so I'm just going to assume it works.
        // And I had to translate it from C++ to Java, so it might not work.
        // Especially the write Functions as in the source code the write function had a count but here there is none... 
        if (mpu6050.writeBytes(MPU6050_RA_BANK_SEL, tmp, 2)) return true;

        if (mpu6050.writeBytes(MPU6050_RA_MEM_R_W,  tmp2)) return true;
        
        return false;
    }
    
    /**
     * Read from the DMP memory.
     * @param mem_addr Memory location (bank << 8 | start address)
     * @param length Number of bytes to read.
     * @return Data read from memory Null if aborted.
     */
    protected byte[] readMem(short mem_addr, short length) {
        byte[] tmp = new byte[2];
        byte[] buffer = new byte[length];
        
        tmp[0] = (byte) (mem_addr >> 8);
        tmp[1] = (byte) (mem_addr & 0xFF);

        if (tmp[1] + length > MPU6050_DMP_MEMORY_BANK_SIZE) {
            return null;
        }

        if (mpu6050.writeBytes(MPU6050_RA_BANK_SEL, tmp)) return null;

        buffer = mpu6050.readBytes(MPU6050_RA_MEM_R_W, length);

        return buffer;
    }
    

    /**
     * Initilizes the MPU6050.
     * Will be called in the constructor.
     */
    abstract void initialize();
}