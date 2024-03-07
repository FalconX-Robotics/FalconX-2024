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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardHelper;
import frc.robot.DashboardHelper.LogLevel;

public class Vision extends SubsystemBase {

    PhotonCamera m_camera = new PhotonCamera("Shelldon");
   
    public LEDs.Color ledsIsAligned = LEDs.Color.HEARTBEAT_BLUE;
    public LEDs ledColor = new LEDs();

    public Optional<Double> getAngleToTarget() {

    var result = m_camera.getLatestResult();
    
    DashboardHelper.putNumber(LogLevel.Debug,"ambiguity", result.getMultiTagResult().estimatedPose.ambiguity);
    DashboardHelper.putBoolean(LogLevel.Debug,"estimated pose is present", result.getMultiTagResult().estimatedPose.isPresent);
    // SmartDashboard.putNumberArray("detected ids", result.getMultiTagResult().fiducialIDsUsed.toArray(new Double[result.getMultiTagResult().fiducialIDsUsed.size()]));
    // result.getTargets().
        // Find distance between targets and camera
         if (result.getMultiTagResult().estimatedPose.isPresent) {

            Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
            Transform3d blueTargetToField = new Transform3d(new Translation3d(0, -5.5, -2), new Rotation3d()); // Blue Speakers
            Transform3d redTargetToField = new Transform3d(new Translation3d(0., 0., 0.), new Rotation3d()); //coordinates unknown at this time (Red Speakers)
            Transform3d blueTargetToCamera = blueTargetToField.plus(fieldToCamera);
            Transform3d redTargetToCamera = redTargetToField.plus(fieldToCamera);

            var alliance = DriverStation.getAlliance();


            if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                DashboardHelper.putNumber(LogLevel.Info,"PV TargetX", blueTargetToCamera.getX());
                DashboardHelper.putNumber(LogLevel.Info,"PV TargetY", blueTargetToCamera.getY()); 
                DashboardHelper.putNumber(LogLevel.Info,"PV TargetZ", blueTargetToCamera.getZ()); 
            } else {
                DashboardHelper.putNumber(LogLevel.Info,"PV TargetX", redTargetToCamera.getX());
                DashboardHelper.putNumber(LogLevel.Info,"PV TargetY", redTargetToCamera.getY()); 
                DashboardHelper.putNumber(LogLevel.Info,"PV TargetZ", redTargetToCamera.getZ()); 
            }          

            Rotation3d m_rotation = blueTargetToCamera.getRotation();

            if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                m_rotation = redTargetToCamera.getRotation();
            }

            

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
        Transform3d result = new Transform3d(new Translation3d(0, -5.5, -2), new Rotation3d()); //Translation3d for tags 7 and 8 (Blue side)
        
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

        // public void poseOffsetLedIndicator() {
        //     Optional<Double> angle = getAngleToTarget();
        //     if (angle.isEmpty()) {
        //         DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Target Not Present.");
        //     } else if(Math.abs(angle.get()) < 5) {
        //         ledColor.setColor(ledsIsAligned);
        //         DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Aligned.");
                
        //     } else if(Math.abs(angle.get()) > 5) {
        //         DashboardHelper.putString(LogLevel.Info, "Angle alignment to target", "Unaligned.");
        //     }
        // }

    
}
