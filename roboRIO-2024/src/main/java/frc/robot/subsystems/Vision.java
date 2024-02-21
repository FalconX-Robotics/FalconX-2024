package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public class Vision {

    // TODO why are we removing the camera from robot container??? -w
    private PhotonCamera limelightCamera = new PhotonCamera("limelightCamera");

    // TODO Red speaker
    private Transform3d getSpeakerToFieldTransform() {
        return VisionConstants.BLUE_SPEAKER_TRANSFORM;
    }

    public double getDistanceToSpeaker() {
        var result = limelightCamera.getLatestResult();

        if (result.getMultiTagResult().estimatedPose.isPresent) {
            Transform3d transform = result.getMultiTagResult().estimatedPose.best;
            SmartDashboard.putNumber("PV X", transform.getX());
            SmartDashboard.putNumber("PV Y", transform.getY());
            SmartDashboard.putNumber("PV Z", transform.getZ());
            SmartDashboard.putNumber("PV Rotation Rad", transform.getRotation().getAngle());

            // This is wrong
            Transform3d speakerToRobotTransform = getSpeakerToFieldTransform().plus(transform);

            //speakerToRobotTransform.getRotation().getY()''
            
            // As is this
            // double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS, VisionConstants.TARGET_HEIGHT_METERS, VisionConstants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getMultiTagResult().estimatedPose.best.getPitch()));
            
            // TODO: by william; idk what you're tryin to do but this code i changed looks right now lol
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAMERA_HEIGHT_METERS,
                VisionConstants.TARGET_HEIGHT_METERS,
                VisionConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch())
            );
            return range;
        }
        return 99999;// returns 99999 if nothing?
        
        /* for (PhotonTrackedTarget target:result.getTargets()) {
            if(target.getFiducialId() == )
        
        } */
    }

    public double getAngleToSpeaker() {
        var result = limelightCamera.getLatestResult();

        // TODO: temporary code added by william (best programmer) to remove an error i saw
        // Taylor what is this..
        return 99999999.;
    }
}
