import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import frc.robot.Interphases.PhotonCameraSystem;

class PhotonVisionTest {
    PhotonCameraSystem photonCameraSystem;
    SimPhotonCamera simPhotonCamera;
    SimVisionSystem simVisionSystem;
    double delta = 0.1;

    @BeforeEach
    void setup() {
        photonCameraSystem = new PhotonCameraSystem();
        var camera_details = photonCameraSystem.camera_details;
        simVisionSystem = new SimVisionSystem(camera_details.getCameraName(),170,
         camera_details.getRobotToCam(),10, 1280, 720, 0.01);
        HAL.initialize(500, 0);
    }

    @AfterEach
    void tearDown() throws Exception {
        simVisionSystem.clearVisionTargets();
    }


    @Test
    @Deprecated
    void Test1() {
        photonCameraSystem.getPitch();
        photonCameraSystem.getYaw();
        photonCameraSystem.getArea();
    }

    @Test
    void checkTargetTest() {
        SimVisionTarget simVisionTarget = new SimVisionTarget(new Pose3d(1.5,0.2,0.1, new Rotation3d(0,0,0)), 2, 2.5, 1);
        simVisionSystem.addSimVisionTarget(simVisionTarget);
        simVisionSystem.processFrame(new Pose2d());
        Timer.delay(0.5);
        assertEquals(1, photonCameraSystem.getCurrentAprilTagID());
        assertEquals(5.6, photonCameraSystem.getPitch(), delta);
        assertEquals(11.3, photonCameraSystem.getYaw(), delta);
        System.out.println(photonCameraSystem.getArea());
    }

    @Test
    void checkAllAprilTagIds() {
        for (int i = 1; i <= 8; i++) { // 8 is the max number of tags by default
            simVisionSystem.clearVisionTargets();
            SimVisionTarget simVisionTarget = new SimVisionTarget(new Pose3d(1.5,0.2,0.1, new Rotation3d(0,0,0)), 2, 2.5, i);
            simVisionSystem.addSimVisionTarget(simVisionTarget);
            simVisionSystem.processFrame(new Pose2d());
            assertEquals(i, photonCameraSystem.getCurrentAprilTagID());
        }
        
    }
}
