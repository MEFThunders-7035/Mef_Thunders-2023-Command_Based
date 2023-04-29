import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTrainTest {
    DriveSubsystem driveSubsystem;
    
    @BeforeEach
    void setup() {
        driveSubsystem = new DriveSubsystem(null);
        try {
            Thread.sleep(300);
        } catch (Exception e) {
            DriverStation.reportError("Interrupted", e.getStackTrace());
        }
        HAL.initialize(500, 0);
    }

    @AfterEach
    void tearDown() throws Exception {
        driveSubsystem.stop();
        driveSubsystem.close();
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
    void DriveTrainDriveOverLimitTest() {
        driveSubsystem.drive(2, 2);
        driveSubsystem.drive(-2, 0.5);
        driveSubsystem.stop();
    }
}
