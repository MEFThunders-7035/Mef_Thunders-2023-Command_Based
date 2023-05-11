import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTrainTest {
    DriveSubsystem driveSubsystem;
    private int kdelay = 30;
    
    @BeforeEach
    void setup() {
        driveSubsystem = new DriveSubsystem(null);
        try {
            Thread.sleep(kdelay);
        } catch (Exception e) {
            DriverStation.reportError("Interrupted", e.getStackTrace());
        }
        HAL.initialize(500, 0);
    }

    @AfterEach
    void tearDown() throws Exception {
        driveSubsystem.stop();
        driveSubsystem.close();
        try {
            Thread.sleep(kdelay);
        } catch (Exception e) {
            DriverStation.reportError("Interrupted", e.getStackTrace());
        }
    }

    @Test
    void gyroTest() throws Exception {
        driveSubsystem.calibrateGyro();
        driveSubsystem.getGyroAngle();
        driveSubsystem.getGyroRate();
        driveSubsystem.resetGyro();
    }

    @Test
    void EncoderTest() throws Exception {
        driveSubsystem.resetEncoders();
        driveSubsystem.getLeftEncoderDistance();
        driveSubsystem.getRightEncoderDistance();
        driveSubsystem.getLeftEncoderRate();
        driveSubsystem.getRightEncoderRate();
    }
    
    @Test
    void DrivetrainDriveTest() throws Exception {
        driveSubsystem.drive(1, 1);
        driveSubsystem.drive(-1, -1);
        driveSubsystem.stop();
    }

    @Test
    void DriveTrainSafetyTest() throws Exception {
        driveSubsystem.setEnabled(false);
        driveSubsystem.setEnabled(true);
    }

    @Test
    void DriveTrainDriveOverLimitTest1() throws Exception{
        try {
        driveSubsystem.drive(2, 2);
        throw new Exception("DriveTrainDriveOverLimitTest failed");
        } catch (IllegalArgumentException e) {
            assertEquals("Speed must be between -1 and 1", e.getMessage());
        }
    }

    @Test
    void DriveTrainMotorSetOverLimitTest() throws Exception{
        try {
        driveSubsystem.setMotors(2.1, -2.5);
        throw new Exception("DriveTrainDriveOverLimitTest failed");
        } catch (IllegalArgumentException e) {
            assertEquals("Speed must be between -1 and 1", e.getMessage());
        }
    }

    @Test
    void DriveTrainDriveUnderLimitTest() throws Exception{
        driveSubsystem.drive(0.1, 0.2);
        driveSubsystem.drive(-0.1, -0.1);
        driveSubsystem.stop();
    }
}
