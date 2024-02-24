package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import frc.robot.Constants.VisionConstants;

public class Vision {

    public Optional<Double> getAngleToTarget() {

        PhotonCamera camera = new PhotonCamera("photonvision");

        var result = camera.getLatestResult();

        // Find distance between targets and camera
        if (result.getMultiTagResult().estimatedPose.isPresent) {

            Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
            Transform3d targetToCamera = GetTargetToField().plus(fieldToCamera);

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

    public Transform3d GetTargetToField() {
        Transform3d result = new Transform3d(new Translation3d(0, 5.5, 2), new Rotation3d());

        return result;

        /* AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();  

        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getMultiTagResult(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot); */
    }
}
