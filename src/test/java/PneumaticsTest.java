import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PneumaticsTest {
    PneumaticsSubsystem pneumaticsSubsystem;

    @BeforeEach
    void setup() {
        HAL.initialize(800, 0);
        pneumaticsSubsystem = new PneumaticsSubsystem();
    }

    @AfterEach
    void tearDown() throws Exception {
        pneumaticsSubsystem.close();
    }

    @Test
    void setCompressorTest() throws Exception {
        pneumaticsSubsystem.setCompressor(true);
        pneumaticsSubsystem.setCompressor(false);
        assertEquals(false, pneumaticsSubsystem.getCompressorStateBool());
    }

    @Test
    void setSolenoidTest() throws Exception {
        pneumaticsSubsystem.setSolenoid(true);
        assertEquals(true, pneumaticsSubsystem.getSolenoidStateBool());
        pneumaticsSubsystem.setSolenoid(false);
        assertEquals(false, pneumaticsSubsystem.getSolenoidStateBool());
    }
}
