import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.VerticalElevatorSubsystem;

public class ElevatorsTest {
    VerticalElevatorSubsystem verticalElevatorSubsystem;

    @BeforeEach
    void setup() {
        HAL.initialize(800, 0);
        verticalElevatorSubsystem = new VerticalElevatorSubsystem();
    }

    @AfterEach
    void tearDown() throws Exception {
        verticalElevatorSubsystem.stop();
        verticalElevatorSubsystem.close();
    }

    @Test
    void setVerticalElevatorTest() throws Exception {
        verticalElevatorSubsystem.setMotor(1);
        assertEquals(1, verticalElevatorSubsystem.getLastMotorSet(), 0.001);
        verticalElevatorSubsystem.setMotor(-1);
        assertEquals(-1, verticalElevatorSubsystem.getLastMotorSet(), 0.001);
    }
    
    @Test
    void stopVerticalElevatorTest() throws Exception {
        verticalElevatorSubsystem.setMotor(0.4);
        verticalElevatorSubsystem.stop();
        assertEquals(0, verticalElevatorSubsystem.getLastMotorSet(), 0.001);
    }

    @Test
    void setVerticalElevatorOverLimitTest() {
        try {
            verticalElevatorSubsystem.setMotor(1.1);
        } catch (IllegalArgumentException e) {
            assertEquals("Vertical elevator motor setpoint out of range: 1.1", e.getMessage());
        }
    }
}
