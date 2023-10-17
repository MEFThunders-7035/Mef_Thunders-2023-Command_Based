import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.Redline_IntakeSubsystem;

public class IntakeTests {
    IntakeArmSubsystem neoSubsystem;
    Redline_IntakeSubsystem redlineSubsystem;

    @BeforeEach
    void setup() {
        HAL.initialize(800, 0);
        neoSubsystem = new IntakeArmSubsystem();
        redlineSubsystem = new Redline_IntakeSubsystem();
    }

    @AfterEach
    void tearDown() throws Exception {
        neoSubsystem.stopMotors();
        redlineSubsystem.stopMotor();
        neoSubsystem.close();
        redlineSubsystem.close();
    }

    @Test
    void setNeoTest() throws Exception {
        neoSubsystem.setMotors(1.0);
        assertEquals(1.0, neoSubsystem.getMotorsLastSet());
        neoSubsystem.setMotors(-1.0);
        assertEquals(-1.0, neoSubsystem.getMotorsLastSet());
    }

    @Test
    void stopNeoTest() throws Exception {
        neoSubsystem.setMotors(1.0);
        neoSubsystem.stopMotors();
        assertEquals(0.0, neoSubsystem.getMotorsLastSet());
    }

    @Test
    void NeoSetBrakeModeTest() throws Exception {
        neoSubsystem.setMotorsBrake(false);
        assertEquals(false, neoSubsystem.getMotorsBrakeMode());
        neoSubsystem.setMotorsBrake(true);
        assertEquals(true, neoSubsystem.getMotorsBrakeMode());
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
