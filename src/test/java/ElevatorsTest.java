import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

import frc.robot.subsystems.VerticalElevatorSubsystem;
import frc.robot.subsystems.HorizontalElevatorSubsystem;

public class ElevatorsTest {
    VerticalElevatorSubsystem verticalElevatorSubsystem;
    HorizontalElevatorSubsystem horizontalElevatorSubsystem;

    @BeforeEach
    void setup() {
        HAL.initialize(800, 0);
        verticalElevatorSubsystem = new VerticalElevatorSubsystem();
        horizontalElevatorSubsystem = new HorizontalElevatorSubsystem();
    }

    @AfterEach
    void tearDown() throws Exception {
        verticalElevatorSubsystem.stop();
        horizontalElevatorSubsystem.stop();
        verticalElevatorSubsystem.close();
        horizontalElevatorSubsystem.close();
    }

    @Test
    void setVerticalElevatorTest() throws Exception {
        verticalElevatorSubsystem.setMotor(1);
        assertEquals(1, verticalElevatorSubsystem.getLastMotorSet(), 0.001);
        verticalElevatorSubsystem.setMotor(-1);
        assertEquals(-1, verticalElevatorSubsystem.getLastMotorSet(), 0.001);
    }

    @Test
    void setHorizontalElevatorTest() throws Exception {
        horizontalElevatorSubsystem.setMotor(1);
        assertEquals(1, horizontalElevatorSubsystem.getLastMotorSet(), 0.001);
        horizontalElevatorSubsystem.setMotor(-1);
        assertEquals(-1, horizontalElevatorSubsystem.getLastMotorSet(), 0.001);
    }
    
    @Test
    void stopVerticalElevatorTest() throws Exception {
        verticalElevatorSubsystem.setMotor(0.4);
        verticalElevatorSubsystem.stop();
        assertEquals(0, verticalElevatorSubsystem.getLastMotorSet(), 0.001);
    }

    @Test
    void stopHorizontalElevatorTest() throws Exception {
        horizontalElevatorSubsystem.setMotor(0.4);
        horizontalElevatorSubsystem.stop();
        assertEquals(0, horizontalElevatorSubsystem.getLastMotorSet(), 0.001);
    }

    @Test
    void setHorizontalElevatorOverLimitTest() {
        try {
            horizontalElevatorSubsystem.setMotor(1.1);
        } catch (IllegalArgumentException e) {
            assertEquals("Horizontal elevator motor setpoint out of range: 1.1", e.getMessage());
        }
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
