package frc.robot.MPU6050;

import edu.wpi.first.wpilibj.I2C;

public class BetterI2C extends I2C{
    public BetterI2C(Port port, int deviceAddress) {
        super(port, deviceAddress);
    }

    public boolean writeBits(int register, int bitStart, int length, byte data) {
        byte b = readBytes(register, 1)[0];
        int mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);
        data &= mask;
        b &= ~mask;
        b |= data;
        return write(register, b);
    }

    /**
     * Writes the given data to the sensor.
     * @param register The register to write to.
     * @param data The data to write.
     * @param count The number of bytes to write.
     * @return Transfer Aborted... false for success, true for aborted.
     */
    public boolean writeBytes(int register, byte[] data, int count) {
        byte[] buffer = new byte[count + 1];
        buffer[0] = (byte) register;
        System.arraycopy(data, 0, buffer, 1, count);
        return writeBulk(buffer);
    }

    /**
     * Writes the given data to the sensor.
     * @param register The register to write to.
     * @param data The data to write.
     * @return Transfer Aborted... false for success, true for aborted.
     */
    public boolean writeBytes(int register, byte[] data) {
        return writeBytes(register, data, data.length);
    }

    public boolean writeChars(int register, char[] data) {
        byte[] buffer = new byte[data.length];
        for (int i = 0; i < data.length; i++) {
            buffer[i] = (byte) data[i];
        }
        return writeBytes(register, buffer);
    }
    /**
     * Writes the given data to the sensor.
     * @param register The register to write to.
     * @param data The data to write.
     * @return Transfer Aborted... false for success, true for aborted.
     */
    public boolean writeWord(int register, short data) {
        return writeBytes(register, new byte[] {(byte) (data >> 8), (byte) data});
    }

    /**
     * Writes the given data to the sensor.
     * @param register The register to write to.
     * @param data The data to write.
     * @return Transfer Aborted... false for success, true for aborted.
     */
    public boolean writeWord(int register, int data) {
        return writeWord(register, (short) data);
    }
    
    /**
     * Reads the specified number of bytes from the specified register on the sensor.
     * @param register The register to read from.
     * @param count The number of bytes to read.
     * @return The bytes read from the sensor.
     */
    public byte[] readBytes(int register, int count) {
        byte[] buffer = new byte[count];
        read(register, count, buffer);
        return buffer;
    }

    /**
     * Reads the specified register on the sensor.
     * <p> This is done by using the fact that the sensor will automatically increment the register. </p>
     * @param register The register to read.
     * @return The value read from the sensor.
     */
    public short readShort(int register) {
        byte[] buffer = readBytes(register, 2);
        return (short) ((buffer[0] << 8) | buffer[1]);
    }

    public char[] readChars(int register, int count) {
        char[] buffer = new char[count];
        byte[] temp = readBytes(register, count);
        for (int i = 0; i < count; i++) {
            buffer[i] = (char) temp[i];
        }
        return buffer;
    }
    
    public short[] readWords(int register, int count) {
        short[] buffer = new short[count];
        byte[] temp = readBytes(register, count * 2);
        for (int i = 0; i < count; i++) {
            buffer[i] = (short) ((temp[i * 2] << 8) | temp[i * 2 + 1]);
        }
        return buffer;
    }

    public byte readBits(int register, int bitStart, int length) {
        byte b = readBytes(register, 1)[0];
        int mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        return b;
    }
}
