package frc.robot.MPU6050;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.MPU6050.DMPFirmwareConstant.DMP_FIRMWARE_CHAR;

import java.util.Arrays;

public class MPU6050 implements Gyro{
    private static final byte DEVICE_ADDRESS = 0x68;
    
    // Write registers
    private static final byte SMPLRT_DIV = 0x19;
    private static final byte USER_CTRL = 0x6A;
    private static final byte PWR_MGMT_1 = 0x6B;
    private static final byte INT_PIN_CFG = 0x37;
    private static final byte INT_ENABLE = 0x38;
    private static final byte FIFO_EN = 0x23;
    private static final byte DMP_START = 0x70;
    private static final byte FIFO_COUNTH = 0x72;
    private static final byte FIFO_R_W = 0x74;
    private static final byte CONFIG = 0x1A;
    private static final byte GYRO_CONFIG = 0x1B;
    private static final byte ACCEL_CONFIG = 0x1C;
    private static final byte SIGNAL_PATH_RESET = 0x68;
    private static final char LPF = 0x1A;

    // Bank/DMP Registers
    private static final byte mem_r_w = 0x6F;
    private static final byte bank_sel = 0x6D;
    private static final byte mem_start_addr = 0x6E;
    
    // Read registers
    private static final int TEMP_OUT_H = 0x41;
    private static final int GYRO_XOUT_H = 0x43;
    private static final int GYRO_YOUT_H = 0x45;
    private static final int GYRO_ZOUT_H = 0x47;
    private static final int ACCEL_XOUT_H = 0x3B;
    private static final int ACCEL_YOUT_H = 0x3D;
    private static final int ACCEL_ZOUT_H = 0x3F;
    private static final int WHO_AM_I = 0x75;
    
    // End of Register Map
    
    private static final int bankSize = 256;

    // Constants
    
    private static char DINA0C = 0x0c;
    private static char DINA2C = 0x2c;
    private static char DINA4C = 0x4c;
    private static char DINA6C = 0x6c;
    private static char DINA36 = 0x36;
    private static char DINA46 = 0x46;
    private static char DINA26 = 0x26;
    private static char DINA56 = 0x56;
    private static char DINA66 = 0x66;
    private static char DINA76 = 0x76;
    private static char DINAC9 = 0xc9;
    private static char DINACD = 0xcd;

    private static final int FCFG_1 = 1062;
    private static final int FCFG_2 = 1066;
    private static final int FCFG_3 = 1088;
    private static final int FCFG_7 = 1073;

    private static final short D_0_22 = (22 + 512);
    private static final short D_0_104 = 104;

    private static final int DMP_SAMPLE_RATE = 200;
    private static final int DMP_FEATURE_SEND_RAW_ACCEL = 0x040;
    private static final int DMP_FEATURE_SEND_ANY_GYRO = 0x080; // Raw: 0x080, Calibrated: 0x100
    
    private static final short CFG_6 = 2753;
    private static final short CFG_15 = 2727;
    private static final int CFG_GYRO_RAW_DATA = 2722;
    private static final int CFG_27 = 2742;
    private static final int CFG_20 = 2224;
    private static final int CFG_ANDROID_ORIENT_INT = 1853;
    private static final int CFG_FIFO_ON_EVENT = 2690;

    private static final long GYRO_SF = (46850825L * 200 / DMP_SAMPLE_RATE);

    // Refrecnece: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
    private final I2C mpu6050;
    
    private int dmpPacketSize = 28;

    private byte[] FIFOBuffer = new byte[32];


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

    private short orient;
    private short fifo_rate;

    private boolean was_Connected;
    private boolean dmp_loaded;

    LinearFilter Xfilter;
    LinearFilter Yfilter;
    LinearFilter Zfilter;
    LinearFilter Xaccelfilter;
    LinearFilter Yaccelfilter;
    LinearFilter Zaccelfilter;



    /**
     * Creates a new instance of the MPU6050 class.
     * @param port The I2C port to which the sensor is connected.
     */
    public MPU6050(I2C.Port port) {
        mpu6050 = new I2C(port, DEVICE_ADDRESS);
        initialize();
        /*
        mpu6050.write(PWR_MGMT_1, 0); // Wake up the sensor
        mpu6050.write(PWR_MGMT_1, 0b10000000); // Reset the sensor
        Timer.delay(0.1); // Wait for the sensor to reset just in case
        
        mpu6050.write(PWR_MGMT_1, 0); // Wake up the sensor again, just in case
        mpu6050.write(SIGNAL_PATH_RESET, 0b00000111); // Reset all sensors

        System.out.println(mpu6050.addressOnly()); // Print if the sensor is connected or not, for debugging purposes

        mpu6050.write(GYRO_CONFIG, 0x00); // Set full scale range for gyro
        mpu6050.write(ACCEL_CONFIG, 0x00); // Set full scale range for accelerometer
        mpu6050.write(FIFO_EN, 0x00); // Disable FIFO
        Timer.delay(0.1); // Wait for the reset to complete just in case

        //setup DLPF
        mpu6050.write(CONFIG, 0b00000010); // Set DLPF with 3 ms delay (better to have filtering with some ms)
         */
        
        dmpInitialize();

        LoopTime = 0.005;
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

        was_Connected = false;

        Xfilter = LinearFilter.singlePoleIIR(0.05, LoopTime);
        Yfilter = LinearFilter.singlePoleIIR(0.05, LoopTime);
        Zfilter = LinearFilter.singlePoleIIR(0.05, LoopTime);
        
        Xaccelfilter = LinearFilter.singlePoleIIR(0.05, LoopTime);
        Yaccelfilter = LinearFilter.singlePoleIIR(0.05, LoopTime);
        Zaccelfilter = LinearFilter.singlePoleIIR(0.05, LoopTime);
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

    private boolean write(int register, byte[] data) {
        return write(register, data, data.length);
    }

    private boolean write(int register, byte[] data, int count) {
        byte[] buffer = new byte[count + 1];
        buffer[0] = (byte) register;
        System.arraycopy(data, 0, buffer, 1, count);
        return mpu6050.writeBulk(buffer);
    }

    private boolean writeMem(short mem_addr, int length, char[] data) {
        return writeMem(mem_addr, (short) length, data);
    }
    
    private boolean writeMem(int mem_addr, int length, char[] data) {
        return writeMem((short) mem_addr, (short) length, data);
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
    private boolean writeMem(short mem_addr, short length, char[] data) {
        byte[] tmp = new byte[2];
        byte[] tmp2 = new byte[data.length];
        
        for (int i = 0; i < data.length; i++) {
            tmp2[i] = (byte) data[i];
        }

        tmp[0] = (byte) (mem_addr >> 8);
        tmp[1] = (byte) (mem_addr & 0xFF);

        // Check bank boundaries
        if (tmp[1] + length > bankSize) {
            return true;
        }
        
        // After this point, I couldn't find any documentation on what this code does, so I'm just going to assume it works.
        // And I had to translate it from C++ to Java, so it might not work.
        // Especially the write Functions as in the source code the write function had a count but here there is none... 
        if (write(bank_sel, tmp, 2)) return true;

        if (write(mem_r_w,  tmp2)) return true;
        
        return false;
    }
    
    /**
     * Read from the DMP memory.
     * @param mem_addr Memory location (bank << 8 | start address)
     * @param length Number of bytes to read.
     * @return Data read from memory Null if aborted.
     */
    private byte[] readMem(short mem_addr, short length) {
        byte[] tmp = new byte[2];
        byte[] buffer = new byte[length];
        
        tmp[0] = (byte) (mem_addr >> 8);
        tmp[1] = (byte) (mem_addr & 0xFF);

        if (tmp[1] + length > bankSize) {
            return null;
        }

        if (write(bank_sel, tmp)) return null;

        buffer = read(mem_r_w, length);

        return buffer;
    }
    
    private void initialize() {
        // Reset device
        mpu6050.write(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        Timer.delay(0.1);
        mpu6050.write(PWR_MGMT_1, 0); // Wake up the sensor
    }

    /**
     * Load and verify DMP image.
     * @param length Length of DMP image.
     * @param firmware DMP code.
     * @param start_addr Starting address of DMP code memory.
     * @return Transfer Aborted... false for success, true for aborted.
    */
    private boolean loadDMPFirmware(short length, char firmware[], short start_addr) {
        if (dmp_loaded) return false;
        short ii;
        short this_write;
        // Must divide evenly into bank_size to avoid bank crossings.
        final int LOAD_CHUNK = 16;
        byte[] cur = new byte[LOAD_CHUNK];
        byte[] tmp = new byte[2];

        if (firmware == null) return true;

        for (ii = 0; ii < length; ii += this_write) {
            this_write = (short) Math.min(LOAD_CHUNK, length - ii);
            if (writeMem(ii, this_write, Arrays.copyOfRange(firmware, ii, ii + this_write))) return true;

            cur = readMem(ii, this_write); if (cur == null) return true;

            byte[] tmp2 = new byte[this_write];
            
            for (int i = 0; i < this_write; i++) {
                tmp2[i] = (byte) firmware[ii + i];
            }

            if (Arrays.equals(cur, tmp2)) {
                System.out.println("Passed! " + ii);
            } else {
                DriverStation.reportError("Failed! " + ii,false);
                System.out.println("Failed! " + ii);
                return true;
            }
        }
        tmp[0] = (byte) (start_addr >> 8);
        tmp[1] = (byte) (start_addr & 0xFF);
        if (mpu6050.write(DMP_START, tmp[0])) return true;

        dmp_loaded = true;
        return false;
    }
    
    /**
     * Push gyro and accel orientation to the DMP.
     * The orientation is represented here as the output of
     * inv_orientation_matrix_to_scalar.
     * @param orient Gyro and accel orientation in body frame.
     * @return true if unsuccessful, false if successful.
     */
    public boolean dmpSetOrientation(short orient) {
        char[] gyroRegs = new char[3];
        char[] accelRegs = new char[3];
        char[] gyroRegsTemp = new char[3];
        char[] accelRegsTemp = new char[3];
        final char[] gyroAxes = {DINA4C, DINACD, DINA6C};
        final char[] accelAxes = {DINA0C, DINAC9, DINA2C};
        final char[] gyroSign = {DINA36, DINA56, DINA76};
        final char[] accelSign = {DINA26, DINA46, DINA66};

        gyroRegs[0] = gyroAxes[orient & 3];
        gyroRegs[1] = gyroAxes[(orient >> 3) & 3];
        gyroRegs[2] = gyroAxes[(orient >> 6) & 3];
        accelRegs[0] = accelAxes[orient & 3];
        accelRegs[1] = accelAxes[(orient >> 3) & 3];
        accelRegs[2] = accelAxes[(orient >> 6) & 3];


        for (int i = 0; i < 3; i++) {
            gyroRegsTemp[i] = (char) gyroRegs[i];
            accelRegsTemp[i] = (char) accelRegs[i];
        }
        /* Chip-to-body, axes only. */
        if (writeMem((short)FCFG_1, (short)3, gyroRegsTemp))
            return true;
        if (writeMem((short)FCFG_2, (short)3, accelRegsTemp))
            return true;

        System.arraycopy(gyroSign, 0, gyroRegs, 0, 3);
        System.arraycopy(accelSign, 0, accelRegs, 0, 3);
        
        if ((orient & 4) != 0) {
            gyroRegs[0] |= 1;
            accelRegs[0] |= 1;
        }
        if ((orient & 0x20) != 0) {
            gyroRegs[1] |= 1;
            accelRegs[1] |= 1;
        }
        if ((orient & 0x100) != 0) {
            gyroRegs[2] |= 1;
            accelRegs[2] |= 1;
        }

        for (int i = 0; i < 3; i++) {
            gyroRegsTemp[i] = (char) gyroRegs[i];
            accelRegsTemp[i] = (char) accelRegs[i];
        }

        /* Chip-to-body, sign only. */
        if (writeMem((short) FCFG_3,(short) 3, gyroRegsTemp)) return true;
        
        if (writeMem((short)FCFG_7, (short)3, accelRegsTemp)) return true;
        this.orient = orient;
        return false;
    }
    
    public boolean enableDMP() {
        char[] tmp = new char[10];

        tmp[0] = (char)((GYRO_SF >> 24) & 0xFF);
        tmp[1] = (char)((GYRO_SF >> 16) & 0xFF);
        tmp[2] = (char)((GYRO_SF >> 8) & 0xFF);
        tmp[3] = (char)(GYRO_SF & 0xFF);
        writeMem(D_0_104, 4, tmp);

        tmp[0] = 0xA3; // IDK what this is for... But it's in the source code so I'm just going to leave it here.

        // Send Raw Accel
        tmp[1] = 0xC0;
        tmp[2] = 0xC8;
        tmp[3] = 0xC2;

        // Send Any Gyro
        tmp[4] = 0xC4;
        tmp[5] = 0xCC;
        tmp[6] = 0xC6;
         
        tmp[7] = 0xA3;
        tmp[8] = 0xA3;
        tmp[9] = 0xA3;
        writeMem(CFG_15, 10, tmp);

        /* Don't Send gesture data to the FIFO. */
        tmp[0] = 0xD8;
        writeMem(CFG_27, 1, tmp);

        // Send raw gyro data to the FIFO.
        boolean send_calibrated_gyro = false;
        if (send_calibrated_gyro) {
            tmp[0] = 0xB2;
            tmp[1] = 0x8B;
            tmp[2] = 0xB6;
            tmp[3] = 0x9B;
        } else {
            tmp[0] = 0xC0;
            tmp[1] = 0x80;
            tmp[2] = 0xC2;
            tmp[3] = 0x90;
        }
        writeMem(CFG_GYRO_RAW_DATA, 4, tmp);

        // Disable TAP feature
        tmp[0] = 0xD8;
        writeMem(CFG_20, 1, tmp);

        // Disable Android orientation
        tmp[0] = 0xD8;
        writeMem(CFG_ANDROID_ORIENT_INT, 1, tmp);

        //* Pedometer is Always ON

        // TODO: Add Functions for DMP_FEATURE_LP_QUAT / DM_FEATURE_6X_LP_QUAT
        
        
        // Set Interrupt Mode to continuous
        char[] regs_continuous = {0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9};
        writeMem(CFG_FIFO_ON_EVENT, 11, regs_continuous);

        resetFIFO();

        dmpPacketSize = 0;
        // Send Raw Accel ON:
        dmpPacketSize += 6;
        // Send Any Gyro ON:
        dmpPacketSize += 6;
        // Quaternion ON:
        dmpPacketSize += 16;
        
        return false;
    }
    
    private boolean dmpEnable6xLpQuat(boolean enable) {
        char regs[] = new char[4];
        if (enable) {
            regs[0] = 0x20;
            regs[1] = 0x28;
            regs[2] = 0x30;
            regs[3] = 0x38;
        }
        else {
            regs[0] = 0xA3;
            regs[1] = 0xA3;
            regs[2] = 0xA3;
            regs[3] = 0xA3;
        }
        boolean complete = writeMem(CFG_15, 4, regs);
        resetFIFO();
        return complete;
    }
    
    /**
     * Set DMP output rate.
     * Only used when DMP is on.
     * @param rate Desired fifo rate (Hz).
     * @return Transfer Aborted... false for success, true for aborted.
     */
    private boolean dmpSetFifoRate(short rate) {
        final char[] regsEnd = {0xFE, 0xF2, 0xAB, 0xc4, 0xAA, 0xF1, 0xDF, 0xDF, 0xBB, 0xAF, 0xDF, 0xDF};
        short div;
        char[] tmp = new char[8];

        if (rate > DMP_SAMPLE_RATE) return true;
        
        div = (short) (DMP_SAMPLE_RATE / rate - 1);
        tmp[0] = (char) ((div >> 8) & 0xFF);
        tmp[1] = (char) (div & 0xFF);
        if (writeMem(D_0_22, (short) 2, tmp))
            return true;
        if (writeMem(CFG_6, (short) 12, regsEnd))
            return true;

        this.fifo_rate = rate;
        return false;
    }
    
    private void dmpInitialize() {
        //DMP setup
        mpu6050.write(PWR_MGMT_1, 0x01); //PWR_MGMT_1: reset with 100ms delay
        Timer.delay(0.1);
        /*
        mpu6050.write(PWR_MGMT_1, 0x01); // PLL_XGYRO reference clock
        mpu6050.write(INT_ENABLE, 0x00); // Disable all interrupts
        mpu6050.write(FIFO_EN, 0x01); // MPU FIFO_EN: (all off) Using DMP's FIFO instead
        mpu6050.write(ACCEL_CONFIG,0x00); // ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
        mpu6050.write(INT_PIN_CFG, 0x80); // INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
        mpu6050.write(PWR_MGMT_1, 0x01); // PLL_XGYRO reference clock
        mpu6050.write(SMPLRT_DIV, 0x04); // SMPLRT_DIV: Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
         */
        boolean ran = loadDMPFirmware((short) DMP_FIRMWARE_CHAR.length, DMP_FIRMWARE_CHAR,(short) mem_start_addr); // Load DMP code into memory banks
        if (ran) {
            DriverStation.reportError("Could not load DMP Firmware",false);
            return;
        }
        System.out.println("DMP Firmware Loaded!");

        dmpSetFifoRate((short) 200); // Set DMP output rate to 200Hz
        dmpSetOrientation((short)0);
        dmpEnable6xLpQuat(true); // Enable 6-axis quat
        /*
        mpu6050.write(USER_CTRL, 0x04); // USER_CTRL: Reset Fifo
        mpu6050.write(USER_CTRL, 0x40); // USER_CTRL: Enable Fifo
        mpu6050.write(FIFO_EN, 0x00); //FIFO_EN: Disable all FIFOs, the DMP will use the FIFO instead
        mpu6050.write(INT_ENABLE, 0x02); // INT_ENABLE: RAW_DMP_INT_EN on
         */
        enableDMP();
    }

    public void restartDevice() {
        mpu6050.write(PWR_MGMT_1, 0x80); // PWR_MGMT_1: Device Reset
    }

    public byte getDeviceID() {
        return read(WHO_AM_I, 1)[0];
    }

    public boolean isConnected() {
        return getDeviceID() == 0x68; // 0x34 in the arduino lib for some reason, Check this!
    }

    
    /* FIFO FUNCTIONS */

    private int getFIFOCount() {
        return readShort(FIFO_COUNTH);
    }

    private byte[] getFIFOBytes(int length) {
        return read(FIFO_R_W, length);
    }

    public void resetFIFO() {
        int data;
        data = 0x00;
        // Set ALL Of them to OFF
        mpu6050.write(INT_ENABLE, data); // INT_ENABLE: Disable all interrupts
        mpu6050.write(FIFO_EN, data); // FIFO_EN: Disable all FIFOs
        mpu6050.write(USER_CTRL, data); // USER_CTRL: Disable Fifo

        if (dmp_loaded) {
            data = 0x04 | 0x08; // Reset FIFO and DMP respectively
            mpu6050.write(USER_CTRL, data); // USER_CTRL: Reset FIFO and DMP
            Timer.delay(0.1);
            data = 0x40 | 0x80; // Enable FIFO and DMP respectively
            mpu6050.write(USER_CTRL, data); // USER_CTRL: Enable FIFO and DMP
            data = 0; // Disable all interrupts
            mpu6050.write(INT_ENABLE, data); // INT_ENABLE: Disable all interrupts
            data = 0; // Disable all FIFOs, the DMP will use the FIFO instead
            mpu6050.write(FIFO_EN, data); // FIFO_EN: Disable all FIFOs
        } else {
            mpu6050.write(FIFO_EN, 0x78); // FIFO_EN: Enable all FIFOs, the DMP will use the FIFO instead
            mpu6050.write(INT_ENABLE, 0x02); // INT_ENABLE: RAW_DMP_INT_EN on
        }
    }

    public void resetDMP() {
        mpu6050.write(USER_CTRL, 0x08); // USER_CTRL: Reset DMP
    }

    /* Main Functions */
    /**
     * Closes the connection to the sensor.
     */
    @Override
    public void close() throws Exception {
        mpu6050.close();
    }
    

    private void printFIFOBuffer() {
        int fifoCount = getFIFOCount();
        long[] quat = new long[4];
        int[] accel = new int[3];
        int[] gyro = new int[3];
        int ii = 0;
        SmartDashboard.putNumber("Fifo Count", fifoCount);
        if (fifoCount < dmpPacketSize) return;
        System.out.println("Fifo Count: " + fifoCount);
        FIFOBuffer = getFIFOBytes(dmpPacketSize);
        SmartDashboard.putNumber("Fifo Buffer Lenght", FIFOBuffer.length);
        SmartDashboard.putRaw("Fifo Buffer", FIFOBuffer);

        //LP_QUAT
        quat[0] = ((long)FIFOBuffer[0] << 24) | ((long)FIFOBuffer[1] << 16) |
            ((long)FIFOBuffer[2] << 8) | FIFOBuffer[3];
        quat[1] = ((long)FIFOBuffer[4] << 24) | ((long)FIFOBuffer[5] << 16) |
            ((long)FIFOBuffer[6] << 8) | FIFOBuffer[7];
        quat[2] = ((long)FIFOBuffer[8] << 24) | ((long)FIFOBuffer[9] << 16) |
            ((long)FIFOBuffer[10] << 8) | FIFOBuffer[11];
        quat[3] = ((long)FIFOBuffer[12] << 24) | ((long)FIFOBuffer[13] << 16) |
            ((long)FIFOBuffer[14] << 8) | FIFOBuffer[15];
        ii += 16;
        SmartDashboard.putNumber("Quat 0", quat[0]);
        SmartDashboard.putNumber("Quat 1", quat[1]);
        SmartDashboard.putNumber("Quat 2", quat[2]);
        SmartDashboard.putNumber("Quat 3", quat[3]);
        
        //Raw DATA
        accel[0] = (FIFOBuffer[ii+0] << 8) | FIFOBuffer[ii+1];
        accel[1] = (FIFOBuffer[ii+2] << 8) | FIFOBuffer[ii+3];
        accel[2] = (FIFOBuffer[ii+4] << 8) | FIFOBuffer[ii+5];
        ii += 6;
        gyro[0] = (FIFOBuffer[ii+0] << 8) | FIFOBuffer[ii+1];
        gyro[1] = (FIFOBuffer[ii+2] << 8) | FIFOBuffer[ii+3];
        gyro[2] = (FIFOBuffer[ii+4] << 8) | FIFOBuffer[ii+5];
        ii += 6;
        SmartDashboard.putNumber("Accel X,2", accel[0]);
        SmartDashboard.putNumber("Accel Y,2", accel[1]);
        SmartDashboard.putNumber("Accel Z,2", accel[2]);

        SmartDashboard.putNumber("Gyro X,2", gyro[0]);
        SmartDashboard.putNumber("Gyro Y,2", gyro[1]);
        SmartDashboard.putNumber("Gyro Z,2", gyro[2]);

    }
    
    /**
     * Runs all the calculations to get the angle data, so it's important to run this periodically.
     * @apiNote RUN IT PERIODICALLY. 
     */
    public void update() {
        currentTimestamp = Timer.getFPGATimestamp();
        LoopTime = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;
        
        if (was_Connected != isConnected()) {
            was_Connected = !was_Connected;
            if (was_Connected) {
                System.out.println("MPU6050 Connected!");
            } else {
                System.out.println("MPU6050 Disconnected!");
                return;
            }
        }

        printFIFOBuffer();
        
        // Assuming X axis pointing forward, the Y axis pointing left, and the Z axis pointing up. (Was the case here)
     
        double rateX = this.getRateX();
        double rateY = this.getRateY();
        double rateZ = this.getRate();

        double accelX = this.getAccelX();
        double accelY = this.getAccelY();
        double accelZ = this.getAccelZ();

        double accelRateX = Math.atan2(accelY, accelZ) * 180.0 / Math.PI;
        double accelRateY = Math.atan2(-accelX, Math.sqrt(accelY * accelY + accelZ * accelZ)) * 180 / Math.PI;

        SmartDashboard.putNumber("AccelRateX", accelRateX);
        SmartDashboard.putNumber("AccelRateY", accelRateY);
        
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
                XaccelOffset += getRawAccelX();
                YaccelOffset += getRawAccelY();
                ZaccelOffset += getRawAccelZ();
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
    public double getRoll() {
        return angleX - X_angle_offset;
    }

    /**
     * Return The heading of the Y axis.
     * @return The current pitch angle of the robot in degrees.
     */
    public double getPitch() {
        return angleY - Y_angle_offset;
    }

    @Override
    public double getRate() {
        return getRateZ();
    }

    /**
     * Returns the Rate of the X axis.
     * @return The rate of the sensor in degrees per second.
     */
    private double getRawRateX() {
        return (readShort(GYRO_XOUT_H) / 131.0) - X_rate_offset;
    }

    /**
     * Returns the Raw Rate of the Y axis.
     * @return The rate of the sensor in degrees per second.
     */
    private double getRawRateY() {
        return (-readShort(GYRO_YOUT_H) / 131.0) - Y_rate_offset;
    }

    /**
     * Returns the Raw Rate of the Z axis.
     * @return The rate of the sensor in degrees per second.
     */
    private double getRawRateZ() {
        /**
        byte[] buffer = new byte[6];
        mpu6050.read(GYRO_ZOUT_H, 6, buffer);
        int z = (buffer[4] << 8) | buffer[5];
        return ((double) z / 131.0) - rate_offset;
         */
        return (-readShort(GYRO_ZOUT_H) / 131.0) - rate_offset;
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getRateX() {
        return getRawRateX();
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getRateY() {
        return getRawRateY();
    }

    /**
     * Gets the angle of the sensor.
     * @return The rate of the sensor in degrees per second.
     */
    public double getRateZ() {
        return getRawRateZ();
    }

    
    private double getRawAccelX() {
        return (readShort(ACCEL_XOUT_H) / 16384.0) - X_Accel_offset;
    }

    private double getRawAccelY() {
        return (readShort(ACCEL_YOUT_H)  / 16384.0) - Y_Accel_offset;
    }

    private double getRawAccelZ() {
        return (readShort(ACCEL_ZOUT_H)/ 16384.0) - Z_Accel_offset ;
    }
    
    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelX() {
        return Xaccelfilter.calculate(getRawAccelX());
    }

    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelY() {
        return Yaccelfilter.calculate(getRawAccelY());
    }
    
    /**
     * Gets the Acceleration of the sensor.
     * @return The Acceleration of the sensor in meters per second squared.
     */
    public double getAccelZ() {
        return Zaccelfilter.calculate(getRawAccelZ());
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
