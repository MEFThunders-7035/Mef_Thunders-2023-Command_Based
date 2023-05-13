package frc.robot.Interphases;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Gyro;
public class ArduinoGyro implements Gyro{
    private final SerialPort ArduinoGyro;
    private String LastRecivedData;
    
    public ArduinoGyro(){
        this.ArduinoGyro = new SerialPort(115200, SerialPort.Port.kUSB);
    }

    @Override
    public double getAngle() throws RuntimeException{
        ArduinoGyro.write(new byte[] {0x11}, 1);
        if (LastRecivedData == ArduinoGyro.readString()) {
            throw new RuntimeException("ArduinoGyro did not give new information or you sent the same command twice!");
        }
        LastRecivedData = ArduinoGyro.readString();
        if (LastRecivedData.isBlank()) {
            throw new RuntimeException("No data recived!");
        }
        return Double.parseDouble(ArduinoGyro.readString());
    }

    @Override
    public void calibrate() {
        ArduinoGyro.write(new byte[] {0x13}, 1);
    }

    @Override
    public void close() throws Exception {
        ArduinoGyro.close();
    }

    @Override
    public double getRate() throws RuntimeException{
        ArduinoGyro.write(new byte[] {0x14}, 1);
        if (LastRecivedData == ArduinoGyro.readString()) {
            throw new RuntimeException("ArduinoGyro did not give new information or you sent the same command twice!");
        }
        LastRecivedData = ArduinoGyro.readString();
        if (LastRecivedData.isBlank()) {
            throw new RuntimeException("No data recived!");
        }
        return Double.parseDouble(ArduinoGyro.readString());
    }

    @Override
    public void reset() {
        ArduinoGyro.write(new byte[] {0x12}, 1);        
    }
    
    public boolean isConnected() throws RuntimeException{
        ArduinoGyro.write(new byte[] {0x10}, 1);
        LastRecivedData = ArduinoGyro.readString();
        if (LastRecivedData.isBlank()) {
            return false;
        }
        return ArduinoGyro.readString() == "Pong!";
    }

}
