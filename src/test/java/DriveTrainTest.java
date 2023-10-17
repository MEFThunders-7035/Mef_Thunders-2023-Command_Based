import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.simulation.EncoderSim;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTrainTest {
    double delta = 0.2;
    DriveSubsystem driveSubsystem;
    EncoderSim sim_leftEncoder;
    EncoderSim sim_rightEncoder;
    
    @BeforeEach
    void setup() {
        driveSubsystem = new DriveSubsystem(null);
        sim_leftEncoder = new EncoderSim(driveSubsystem.getLeftEncoder());
        sim_rightEncoder = new EncoderSim(driveSubsystem.getRightEncoder());
        HAL.initialize(500, 0);
    }

    @AfterEach
    void tearDown() throws Exception {
        driveSubsystem.stopMotors();
        driveSubsystem.close();
    }

    @Test
    void gyroTest() throws Exception {
        driveSubsystem.calibrateGyro();
        driveSubsystem.getAngle();
        driveSubsystem.getRotationRate();
        driveSubsystem.resetGyro();
    }

    @Test
    void EncoderResetTest() throws Exception {
        sim_leftEncoder.setDistance(5);
        sim_rightEncoder.setDistance(5);
        driveSubsystem.resetEncoders();
        assertEquals(0, driveSubsystem.getLeftEncoderDistance(), delta);
    }

    @Test
    void EncoderDistanceTest() throws Exception {
        sim_leftEncoder.setDistance(5);
        sim_rightEncoder.setDistance(5);
        assertEquals(5, driveSubsystem.getLeftEncoderDistance(), delta);
        assertEquals(driveSubsystem.getRightEncoderDistance(), 5, delta);
    }

    @Test
    void EncoderRateTest() throws Exception {
        sim_leftEncoder.setRate(5);
        sim_rightEncoder.setRate(5);
        assertEquals(5, driveSubsystem.getLeftEncoderRate(), delta);
        assertEquals(driveSubsystem.getRightEncoderRate(), 5, delta);
    }
    
    @Test
    void DrivetrainFowardTest() throws Exception {
        driveSubsystem.drive(1, 0);
        driveSubsystem.drive(-1, 0);
        driveSubsystem.stopMotors();
    }

    @Test
    void StopTest() throws Exception {
        driveSubsystem.drive(1, 0);
        driveSubsystem.stopMotors();
    }

    @Test
    void DriveTrainTurnTest() throws Exception {
        driveSubsystem.drive(0, 1);
    }
}
