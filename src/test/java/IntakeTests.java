import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.Neo_IntakeSubsystem;
import frc.robot.subsystems.Redline_IntakeSubsystem;

public class IntakeTests {
    Neo_IntakeSubsystem neoSubsystem;
    Redline_IntakeSubsystem redlineSubsystem;

    @BeforeEach
    void setup() {
        HAL.initialize(800, 0);
        neoSubsystem = new Neo_IntakeSubsystem();
        redlineSubsystem = new Redline_IntakeSubsystem();
    }

    @AfterEach
    void tearDown() throws Exception {
        neoSubsystem.stopNeo();
        redlineSubsystem.stopMotor();
        neoSubsystem.close();
        redlineSubsystem.close();
    }

    @Test
    void setNeoTest() throws Exception {
        neoSubsystem.setNeo(1.0);
        assertEquals(1.0, neoSubsystem.getNeoLastSet());
        neoSubsystem.setNeo(-1.0);
        assertEquals(-1.0, neoSubsystem.getNeoLastSet());
    }

    @Test
    void stopNeoTest() throws Exception {
        neoSubsystem.setNeo(1.0);
        neoSubsystem.stopNeo();
        assertEquals(0.0, neoSubsystem.getNeoLastSet());
    }

    @Test
    void NeoSetBrakeModeTest() throws Exception {
        neoSubsystem.setNeoBrake(false);
        assertEquals(false, neoSubsystem.getNeoBrakeMode());
        neoSubsystem.setNeoBrake(true);
        assertEquals(true, neoSubsystem.getNeoBrakeMode());
    }

    @Test
    void setNeoCurrentLimitTest() {
        neoSubsystem.setNeoCurrentLimit(40);
    }

    @Test
    void setNeoRampRateTest() {
        neoSubsystem.setNeoRampRate(0.5);
        assertEquals(0.5, neoSubsystem.getNeoRampRate());
    }

    @Test
    void setNeoOverLimitTest() {
        try {
            neoSubsystem.setNeo(1.5);
        } catch (IllegalArgumentException e) {
            assertEquals("Neo setpoint out of range: 1.5", e.getMessage());
        }
    }

    @Test
    void setRedlineTest() throws Exception {
        redlineSubsystem.setRedline(1.0);
        assertEquals(1.0, redlineSubsystem.getLastMotorSet());
        redlineSubsystem.setRedline(-1.0);
        assertEquals(-1.0, redlineSubsystem.getLastMotorSet());
    }

    @Test
    void setRedlineOverLimitTest() {
        try {
            redlineSubsystem.setRedline(1.5);
        } catch (IllegalArgumentException e) {
            assertEquals("Redline motor setpoint out of range: 1.5", e.getMessage());
        }
    }
}
