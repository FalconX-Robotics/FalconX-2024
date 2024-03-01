package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants.VisionConstants;

public class Vision {

    PhotonCamera m_camera = new PhotonCamera("Shelldon");

    public Optional<Double> getAngleToTarget() {

    var result = m_camera.getLatestResult();
    
    SmartDashboard.putNumber("ambiguity", result.getMultiTagResult().estimatedPose.ambiguity);
    SmartDashboard.putBoolean("estimated pose is present", result.getMultiTagResult().estimatedPose.isPresent);
    // SmartDashboard.putNumberArray("detected ids", result.getMultiTagResult().fiducialIDsUsed.toArray(new Double[result.getMultiTagResult().fiducialIDsUsed.size()]));
    // result.getTargets().
        // Find distance between targets and camera
        if (result.getMultiTagResult().estimatedPose.isPresent) {

            Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
            Pose3d targetToCamera = getTargetsToField().get(0);


            SmartDashboard.putNumber("PV TargetX", targetToCamera.getX());
            SmartDashboard.putNumber("PV TargetY", targetToCamera.getY()); 
            SmartDashboard.putNumber("PV TargetZ", targetToCamera.getZ()); 
                           
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
}
