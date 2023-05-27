package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase implements AutoCloseable{
    private final DoubleSolenoid double_solenoid;
    private final Compressor compressor;

    public PneumaticsSubsystem() {
        double_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        PneumaticsConstants.kSolenoidFowardPort, 
        PneumaticsConstants.kSolenoidReversePort);

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    @Override
    public void close() throws Exception {
        compressor.close();
        double_solenoid.close();
    }

    public void setCompressor(boolean state) {
        if (state) {
            compressor.disable();
        } else {
            compressor.enableDigital();
        }
    }

    /**
     * Set the solenoid to the state passed in
     * @param state true = forward, false = reverse
     */
    public void setSolenoid(boolean state) {
        if (state) {
            double_solenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            double_solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void stopSelenoid() {
        double_solenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    /**
     * Get the state of the solenoid
     * @return the value of the solenoid in {@link DoubleSolenoid.Value}
     */
    public DoubleSolenoid.Value getSelenoidStateValue() {
        return double_solenoid.get();
    }

    /**
     * Get the state of the solenoid
     * @return true = forward, false = reverse
     */
    public boolean getSolenoidStateBool() throws Exception {
        if (double_solenoid.get().equals(DoubleSolenoid.Value.kForward)) {
            return true;
        } else if (double_solenoid.get().equals(DoubleSolenoid.Value.kReverse)) {
            return false;
        } else if (double_solenoid.get().equals(DoubleSolenoid.Value.kOff)) {
            throw new RuntimeException("Solenoid is off");
        } else {
        }
        throw new Exception("Solenoid is not in a valid state");
    }

    /**
     * Get the state of the compressor
     * @return true = enabled, false = disabled
     */
    public boolean getCompressorStateBool() {
        return compressor.isEnabled();
    }
}
