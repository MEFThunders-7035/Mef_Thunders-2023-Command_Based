import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Constants.VerticalElevatorConstants;
import frc.robot.subsystems.VerticalElevatorSubsystem;

public class ElevatorsTest {
    VerticalElevatorSubsystem verticalElevatorSubsystem;
    double delta = 0.001;
    PWMSim sim_verticalElevatorMotor;

    @BeforeEach
    void setup() {
        sim_verticalElevatorMotor = new PWMSim(VerticalElevatorConstants.kVerticalElevatorPort);
        HAL.initialize(800, 0);
        verticalElevatorSubsystem = new VerticalElevatorSubsystem();
    }

    @AfterEach
    void tearDown() throws Exception {
        verticalElevatorSubsystem.close();
    }

    @Test
    void setVerticalElevatorTest() throws Exception {
        verticalElevatorSubsystem.setMotor(1);
        assertEquals(-1, sim_verticalElevatorMotor.getSpeed(), delta); // motor is inverted
        assertEquals(1, verticalElevatorSubsystem.getLastMotorSet(), delta);
        verticalElevatorSubsystem.setMotor(-1);
        assertEquals(1, sim_verticalElevatorMotor.getSpeed(), delta); // motor is inverted
        assertEquals(-1, verticalElevatorSubsystem.getLastMotorSet(), delta);
    }
    
    @Test
    void stopVerticalElevatorTest() throws Exception {
        verticalElevatorSubsystem.setMotor(0.4);
        verticalElevatorSubsystem.stopMotor();
        assertEquals(0, verticalElevatorSubsystem.getLastMotorSet(), delta);
        assertEquals(0, sim_verticalElevatorMotor.getSpeed(), delta);
    }
}
