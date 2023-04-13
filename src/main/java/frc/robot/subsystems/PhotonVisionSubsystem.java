package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import frc.robot.Constants.PhotonVisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase{
    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    public PhotonVisionSubsystem() {
        camera = new PhotonCamera(PhotonVisionConstants.kCameraName);

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            photonPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, PhotonVisionConstants.robotToCam);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Target Detected", camera.getLatestResult().hasTargets());
    }
    
    public void setLED(VisionLEDMode state) {
        camera.setLED(state);
    }

    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    public double getYaw() {
        return camera.getLatestResult().getBestTarget().getYaw();
    }

    public double getArea() {
        return camera.getLatestResult().getBestTarget().getArea();
    }
    
    public double getPitch() {
        return camera.getLatestResult().getBestTarget().getPitch();
    }
    
    public double getSkew() {
        return camera.getLatestResult().getBestTarget().getSkew();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
