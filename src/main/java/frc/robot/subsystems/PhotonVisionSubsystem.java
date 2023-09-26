package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import frc.robot.Constants.PhotonVisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Deprecated
public class PhotonVisionSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonCamera camera2;
    private PhotonPoseEstimator photonPoseEstimator;
    private Field2d field;
    private double CAMERA_HEIGHT_METERS;
    private double CAMERA_PITCH_RADIANS;

    /**
     * @deprecated Use {@link #PhotonVisionSubsystem(Field2d)} instead.
     * @param field2d The Field2d that will be used to display the robot's pose on the field.
     */
    public PhotonVisionSubsystem(Field2d field2d) {
        if (field2d == null) {
            field2d = new Field2d();
        }
        this.field = field2d;
        SmartDashboard.putData("Field", field);
        camera = new PhotonCamera(PhotonVisionConstants.Cameras.kPiCamera);
        camera2 = new PhotonCamera(PhotonVisionConstants.Cameras.kWideCamera);
        camera2.setDriverMode(true);
        setCurrentCameraHeightAndPitch();
        System.out.println("Loaded PhotonCamera, Added Field to SmartDashboard");
        photonPoseEstimator = getPhotonPoseEstimator();
    }

    public PhotonVisionSubsystem(Field2d field, String Camera_Name) {
        if (field == null) {
            field = new Field2d();
        }
        this.field = field;
        SmartDashboard.putData("Field", field);
        setCamera(Camera_Name);
        setCurrentCameraHeightAndPitch();
        
        System.out.println("Loaded PhotonCamera, Added Field to SmartDashboard");
        photonPoseEstimator = getPhotonPoseEstimator();
    }
    
    
    @Override
    public void periodic() {
        if (hasTargets()) {
            var estimated_global_pose = getEstimatedGlobalPose(field.getRobotPose());
            if (estimated_global_pose.isPresent()) {
                field.setRobotPose(estimated_global_pose.get().estimatedPose.toPose2d());
            }
        }

    }
    
    /**
     * @return return if the current Camera is the picam.
     */
    public boolean isPicam() {
        return camera.getName() == PhotonVisionConstants.Cameras.kPiCamera;
    }

    private void setCurrentCameraHeightAndPitch() {
        if (isPicam()) {
            CAMERA_HEIGHT_METERS = PhotonVisionConstants.PiCamera.CAMERA_HEIGHT_METERS;
            CAMERA_PITCH_RADIANS = PhotonVisionConstants.PiCamera.CAMERA_PITCH_RADIANS;
        } else {
            CAMERA_HEIGHT_METERS = PhotonVisionConstants.WideCamera.kCamera_Height_Meters;
            CAMERA_PITCH_RADIANS = PhotonVisionConstants.WideCamera.kCamera_Pitch_Radians;
        }
    }

    /**
     * @return returns the current camera's 3d transform.
     */
    private Transform3d getCurrentTransform3d() {
        if (isPicam()) {
            return PhotonVisionConstants.PiCamera.robotToCam;
        } else {
            return PhotonVisionConstants.WideCamera.robotToCam;
        }
    }
    
    /**
     * has the Turn PID constants for the current camera. Use {@link #getkP()} , {@link #getkI()}, and {@link #getkD()} to get the constants.
     */
    public class CurrentTurnPIDConstants {

        /**
         * @return returns the kP constant for the current camera.
         */
        public double getkP() {
            return getCurrentTurnPIDConstants()[0];
        }
        
        /**
         * @return returns the kI constant for the current camera.
         */
        public double getkI() {
            return getCurrentTurnPIDConstants()[1];
        }

        /**
         * @return returns the kD constant for the current camera.
         */
        public double getkD() {
            return getCurrentTurnPIDConstants()[2];
        }
    }

    /**
     * @return returns the turn PID constants for the current camera. [0] = kP, [1] = kI, [2] = kD.
     */
    public double[] getCurrentTurnPIDConstants() {
        if (isPicam()) {
            return new double[] {PhotonVisionConstants.PiCamera.TurnPIDConstants.kP, PhotonVisionConstants.PiCamera.TurnPIDConstants.kI, PhotonVisionConstants.PiCamera.TurnPIDConstants.kD};
        } else {
            return new double[] {PhotonVisionConstants.WideCamera.TurnPIDConstants.kP, PhotonVisionConstants.WideCamera.TurnPIDConstants.kI, PhotonVisionConstants.WideCamera.TurnPIDConstants.kD};
        }
    }
    
    /**
     * Has the foward PID constants for the current camera. Use {@link #getkP()} , {@link #getkI()}, and {@link #getkD()} to get the constants.
     */
    public class CurrentFowardPIDConstants {

        /**
         * @return returns the kP constant for the current camera.
         */
        public double getkP() {
            return getCurrentFowardPIDConstants()[0];
        }
        
        /**
         * @return returns the kI constant for the current camera.
         */
        public double getkI() {
            return getCurrentFowardPIDConstants()[1];
        }

        /**
         * @return returns the kD constant for the current camera.
         */
        public double getkD() {
            return getCurrentFowardPIDConstants()[2];
        }
    }

    /**
     * @return returns the foward PID constants for the current camera. [0] = kP, [1] = kI, [2] = kD.
     */
    public double[] getCurrentFowardPIDConstants() {
        if (isPicam()) {
            return new double[] {PhotonVisionConstants.PiCamera.FowardPIDConstants.kP, PhotonVisionConstants.PiCamera.FowardPIDConstants.kI, PhotonVisionConstants.PiCamera.FowardPIDConstants.kD};
        } else {
            return new double[] {PhotonVisionConstants.WideCamera.FowardPIDConstants.kP, PhotonVisionConstants.WideCamera.FowardPIDConstants.kI, PhotonVisionConstants.WideCamera.FowardPIDConstants.kD};
        }
    }

    /**
     * @return the current id of the best april tag being tracked. If no tag is being tracked, it will return -1.
     */
    public int getCurrentAprilTagID() {
        var latest_result = camera.getLatestResult();
        if (latest_result.hasTargets()) {
            return latest_result.getBestTarget().getFiducialId();
        }
        return -1;
    }
    
    /**
     * @return the current ids of each apriltag being tracked. If there are no aprilTags is being tracked, it will return an empty array.
     */
    public List<Integer> getTrackedTargets() {
        List<Integer> ids = new ArrayList<Integer>();
        if (!camera.getLatestResult().hasTargets()) return ids;
        var targets = camera.getLatestResult().getTargets();
        for (var target : targets) {
            ids.add(target.getFiducialId());
        }
        return ids;
    }

    /**
     * Allows you to set camera used to detect the targets.
     * @param Camera the name of the camera you want to use. Use {@link PhotonVisionConstants.Cameras}
     */
    public void setCamera(String Camera) {
        camera = new PhotonCamera(Camera);
        camera.setDriverMode(false);
        
        if (Camera == PhotonVisionConstants.Cameras.kPiCamera) {
            camera2 = new PhotonCamera(PhotonVisionConstants.Cameras.kWideCamera);
            camera2.setDriverMode(true);
        }
        
        else if (Camera == PhotonVisionConstants.Cameras.kWideCamera) {
            camera2 = new PhotonCamera(PhotonVisionConstants.Cameras.kPiCamera);
            camera2.setDriverMode(true);
        }

        else {
            DriverStation.reportError("Unknown Camera Name Please use PhotonVisionConstants.Cameras", null);
        }
    }

    /**
     * disables the Driver Mode on the PhotonVision camera.
     */
    public void enableSecondCamera() {
        camera2.setDriverMode(false);
    }

    /**
     * puts the second camera to driver mode.
     */
    public void disableSecondCamera() {
        camera2.setDriverMode(true);
    }

    /**
     * @return the Second Camera.
     */
    public PhotonCamera getSecondCamera() {
        return camera2;
    }
    
    /**
     * Sets the LED mode of the PhotonVision camera.
     * @param state The LED mode to set.
     */
    public void setLED(VisionLEDMode state) {
        camera.setLED(state);
    }

    /**
     * Sets the pipeline of the PhotonVision camera.
     * @param pipeline The pipeline to set.
     */
    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    /**
     * Gets the Yaw of the target, acording to the FOV of the camera.
     * @return The Yaw of the target.
     */
    public double getYaw() {
        if (!hasTargets()) {
            DriverStation.reportWarning("No Target Found", false);
            return 0;
        }
        if (hasTargets()) return camera.getLatestResult().getBestTarget().getYaw();

        return 0;
    }

    /**
     * Gets the Area of the target, acording to the area of the camera's FOV.
     * @return The Area of the target.
     */
    public double getArea() {
        if (!hasTargets()) {
            DriverStation.reportWarning("No Target Found", false);
            return 0;
        }
        return camera.getLatestResult().getBestTarget().getArea();
    }
    
    public double getPitch() {
        if (!hasTargets()) {
            DriverStation.reportWarning("No Target Found", false);
            return 0;
        }
        return camera.getLatestResult().getBestTarget().getPitch();
    }
    
    public double getSkew() {
        if (!hasTargets()) {
            DriverStation.reportWarning("No Target Found", false);
            return 0;
        }
        return camera.getLatestResult().getBestTarget().getSkew();
    }
    
    public PhotonCamera getCamera() {
        return camera;
    }
    
    public Boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Finds the {@link Pose2d} using aprilTags.
     * @param prevEstimatedRobotPose The previous estimated global pose of the robot in {@link Pose2d}.
     * @return The new {@link EstimatedRobotPose} To get {@link Pose2d} use {@code EstimatedRobotPose.get().estimatedPose.toPose2d()}.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
    
    /**
     * Gets the distance to the target in meters.
     * Use {@link #getEstimatedGlobalPose(Pose2d)} to get the robot position, as it is more accurate.
     * @return
     */
    public double getDistance() {
        return PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, PhotonVisionConstants.TARGET_HEIGHT_METERS, -CAMERA_PITCH_RADIANS, getPitch());
    }


    private PhotonPoseEstimator getPhotonPoseEstimator() {
        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            photonPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, getCurrentTransform3d());
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            System.out.println("Loaded PhotonPoseEstimator");
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
        return photonPoseEstimator;
    }
        
}
