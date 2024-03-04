package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DashboardHelper;
import frc.robot.DashboardHelper.LogLevel;

public class Vision {

    PhotonCamera m_camera = new PhotonCamera("Shelldon");
    LEDs.Color ledsIsNotAligned = LEDs.Color.HEARTBEAT_RED;
    LEDs.Color ledsIsAligned = LEDs.Color.HEARTBEAT_BLUE;
    LEDs ledColor = new LEDs();

    public Optional<Double> getAngleToTarget() {

    var result = m_camera.getLatestResult();
    
    DashboardHelper.putNumber(LogLevel.Debug,"ambiguity", result.getMultiTagResult().estimatedPose.ambiguity);
    DashboardHelper.putBoolean(LogLevel.Debug,"estimated pose is present", result.getMultiTagResult().estimatedPose.isPresent);
    // SmartDashboard.putNumberArray("detected ids", result.getMultiTagResult().fiducialIDsUsed.toArray(new Double[result.getMultiTagResult().fiducialIDsUsed.size()]));
    // result.getTargets().
        // Find distance between targets and camera
        if (result.getMultiTagResult().estimatedPose.isPresent) {

            Pose3d targetToCamera = getTargetsToField().get(0);


            DashboardHelper.putNumber(LogLevel.Info,"PV TargetX", targetToCamera.getX());
            DashboardHelper.putNumber(LogLevel.Info,"PV TargetY", targetToCamera.getY()); 
            DashboardHelper.putNumber(LogLevel.Info,"PV TargetZ", targetToCamera.getZ()); 
                           
            Rotation3d m_rotation = targetToCamera.getRotation();
            return Optional.of(m_rotation.getZ());

            /* double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAMERA_HEIGHT_METERS,
                VisionConstants.TARGET_HEIGHT_METERS,
                VisionConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.MultitagResult())); */
        }

        return Optional.empty();
    }

    public ArrayList<Pose3d> getTargetsToField() {
        Transform3d result = new Transform3d(new Translation3d(0, 5.5, 2), new Rotation3d());
        
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();  
        ArrayList<Pose3d> pose3ds = new ArrayList<>();
        m_camera.getLatestResult().targets.forEach((target) -> {
            
            pose3ds.add(aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());
        });

        return pose3ds;

        // PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        // , m_camera, result);

        
        // return result;
        /*Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getMultiTagResult(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot); */
    }
    // public boolean poseOffesetLedIndicator(boolean isAligned) {
    //     if (getTargetsToField().get(0).getX() < 3 /*&& get distanve here < 2 meters*/ ) {
    //         return true;
    //     }
    //     return false;
    // }

        public void poseOffsetLedIndicator() {
            if (getTargetsToField().get(0).getX() < 3 ) {
                ledColor.setColor(ledsIsAligned);
                DashboardHelper.putBoolean(LogLevel.Info, "Robot Alignment to Target", true);
                // Commands.run(()->{
                //     ledColor.setColor(ledsIsAligned);
                // }, ledColor);
            } else {
                ledColor.setColor(ledsIsNotAligned);
                DashboardHelper.putBoolean(LogLevel.Info, "Robot Alignment to Target", false);
                

            }
        }

    
}
