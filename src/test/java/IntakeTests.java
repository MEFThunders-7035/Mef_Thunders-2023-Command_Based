import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.Redline_IntakeSubsystem;

public class IntakeTests {
    IntakeArmSubsystem IntakeArmsubsystem;
    Redline_IntakeSubsystem redlineSubsystem;

    @BeforeEach
    void setup() {
        HAL.initialize(800, 0);
        IntakeArmsubsystem = new IntakeArmSubsystem();
        redlineSubsystem = new Redline_IntakeSubsystem();
    }

    @AfterEach
    void tearDown() throws Exception {
        IntakeArmsubsystem.stopMotors();
        redlineSubsystem.stopMotor();
        IntakeArmsubsystem.close();
        redlineSubsystem.close();
    }

    @Test
    void setNeoTest() throws Exception {
        IntakeArmsubsystem.setMotors(1.0);
        assertEquals(1.0, IntakeArmsubsystem.getMotorsLastSet());
        IntakeArmsubsystem.setMotors(-1.0);
        assertEquals(-1.0, IntakeArmsubsystem.getMotorsLastSet());
    }

    @Test
    void stopNeoTest() throws Exception {
        IntakeArmsubsystem.setMotors(1.0);
        IntakeArmsubsystem.stopMotors();
        assertEquals(0.0, IntakeArmsubsystem.getMotorsLastSet());
    }

    @Test
    void NeoSetBrakeModeTest() throws Exception {
        IntakeArmsubsystem.setMotorsBrake(false);
        assertEquals(false, IntakeArmsubsystem.getMotorsBrakeMode());
        IntakeArmsubsystem.setMotorsBrake(true);
        assertEquals(true, IntakeArmsubsystem.getMotorsBrakeMode());
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
