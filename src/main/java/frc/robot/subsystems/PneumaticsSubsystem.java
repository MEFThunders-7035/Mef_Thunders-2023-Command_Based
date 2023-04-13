package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase{
    private final DoubleSolenoid double_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,PneumaticsConstants.kSolenoidFowardPort, PneumaticsConstants.kSolenoidReversePort);
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public PneumaticsSubsystem() {
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
}
