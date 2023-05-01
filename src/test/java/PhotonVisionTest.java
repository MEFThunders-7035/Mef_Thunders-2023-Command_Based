import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.PhotonVisionSubsystem;

class PhotonVisionTest {
    PhotonVisionSubsystem photonVisionSubsystem;

    @BeforeEach
    void setup() {
        HAL.initialize(500, 0);
        photonVisionSubsystem = new PhotonVisionSubsystem(null);
    }

    @AfterEach
    void tearDown() throws Exception {}

    @Test
    void RunsWhenNoCameraConnected() {
        // this test will pass if a camera is not connected and the code runs
        // this test will fail if a camera is not connected and the code does not run

        photonVisionSubsystem.getCamera();
        photonVisionSubsystem.periodic();
        photonVisionSubsystem.getDistance();
        photonVisionSubsystem.getPitch();
        photonVisionSubsystem.getYaw();
        photonVisionSubsystem.getArea();
        photonVisionSubsystem.getSkew();
        photonVisionSubsystem.getEstimatedGlobalPose(null);
    }    
}
