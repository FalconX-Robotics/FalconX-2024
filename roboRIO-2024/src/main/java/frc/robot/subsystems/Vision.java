package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardHelper;
import frc.robot.DashboardHelper.LogLevel;

public class Vision extends SubsystemBase {

    // X - Forward (Normal to plane)
    // Y - Right
    // Z - Up
    // Origin at center of tag

    LEDs m_leds;
    public Vision(LEDs leds) {
        m_leds = leds;
    }

    PhotonCamera m_camera = new PhotonCamera("Shelldon");
    PhotonPipelineResult latestResult;

    //public LEDs.Color ledsIsAligned = LEDs.Color.HEARTBEAT_BLUE;

    public Optional<Double> getAngleToSpeaker() {

        var result = m_camera.getLatestResult();

        List<PhotonTrackedTarget> targets = result.getTargets();

        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == 4 || target.getFiducialId() == 7) {
                if (target.getPoseAmbiguity() < 0.2) {
                    DashboardHelper.putNumber(LogLevel.Debug, "yaw", target.getYaw());
                    DashboardHelper.putBoolean(LogLevel.Debug, "tag detected", true);
                    target.getAlternateCameraToTarget();
                    return Optional.of(Math.atan(Units.Inches.toBaseUnits(target.getBestCameraToTarget().getY()+15)
                    /(Units.Inches.toBaseUnits(target.getBestCameraToTarget().getZ())-15))
                    );
                }
            }
            DashboardHelper.putBoolean(LogLevel.Debug, "tag detected", false);
            return Optional.empty();
        }

        return Optional.empty(); 
    }


    // public Optional<Double> getAngleToTarget() {
    // public Optional<Double> getAngleToTarget() {

    //     var result = m_camera.getLatestResult();
    //     List<PhotonTrackedTarget> targets = result.getTargets();
    //     var result = m_camera.getLatestResult();
    //     List<PhotonTrackedTarget> targets = result.getTargets();

    //     for (PhotonTrackedTarget target : targets) {
    //         if (target.getFiducialId() == 4 || target.getFiducialId() == 7) {
    //             // if (target.getPoseAmbiguity() < 0.1) {
    //                 DashboardHelper.putNumber(LogLevel.Debug, "yaw", target.getYaw());
    //                 return Optional.of(target.getPitch());
    //             // }
    //         }
    //         DashboardHelper.putBoolean(LogLevel.Debug, "tag detected", false);
    //         return Optional.empty();
    //     }
    //     for (PhotonTrackedTarget target : targets) {
    //         if (target.getFiducialId() == 4 || target.getFiducialId() == 7) {
    //             // if (target.getPoseAmbiguity() < 0.1) {
    //                 DashboardHelper.putNumber(LogLevel.Debug, "yaw", target.getYaw());
    //                 return Optional.of(target.getPitch());
    //             // }
    //         }
    //         DashboardHelper.putBoolean(LogLevel.Debug, "tag detected", false);
    //         return Optional.empty();
    //     }

    //     return Optional.empty(); 
    //     return Optional.empty(); 

    /*double angle = Math.atan(Units.Inches.toBaseUnits(target.getBestCameraToTarget().getY())
    /*double angle = Math.atan(Units.Inches.toBaseUnits(target.getBestCameraToTarget().getY())
                                           /(Units.Inches.toBaseUnits(target.getBestCameraToTarget().getZ()+15)));
                     */

    public ArrayList<Pose3d> getTargetsToField() {
        Transform3d blueResult = new Transform3d(new Translation3d(0, -5.5, 1.5), new Rotation3d()); //Translation3d for tags 7 and 8 (Blue side)
        Transform3d redResult = new Transform3d(new Translation3d(16.5, 5.5, 1.5), new Rotation3d());

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();  
        ArrayList<Pose3d> pose3ds = new ArrayList<>();
        m_camera.getLatestResult().targets.forEach((target) -> {
            
            pose3ds.add(aprilTagFieldLayout.getTagPose(target.getFiducialId()).get());
        });

        return pose3ds;

    }

    private boolean noTarget() {
        var result = m_camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        return targets.isEmpty();
    }

    @Override
    public void periodic () {
        
        var result = m_camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        if (noTarget()) {
            DashboardHelper.putString(LogLevel.Info, "angle to speaker", "No Angle Present");
            DashboardHelper.putString(LogLevel.Info, "distance to target", "No Distnance Present");
        }else{
            // DashboardHelper.putString(LogLevel.Info, "angle to speaker", getAngleToSpeaker().toString());
            // DashboardHelper.putNumber(LogLevel.Debug, "X meters to target", getXMeters().get());
            // DashboardHelper.putNumber(LogLevel.Debug, "Y meters to target", getYMeters().get());
            // DashboardHelper.putNumber(LogLevel.Debug, "Z meters to target", getZMeters().get());
            // DashboardHelper.putString(LogLevel.Info, "distance to target", getDistanceToTargetMeters().toString());
            DashboardHelper.putString(LogLevel.Info, "angle to speaker", getAngleToSpeaker().toString());
            DashboardHelper.putString(LogLevel.Info, "distance to target", getDistanceToTargetMeters().toString());
        }

        DashboardHelper.putBoolean(LogLevel.Debug, "tag detected", !noTarget());

        for (PhotonTrackedTarget target : targets) {
            DashboardHelper.putNumber(LogLevel.Debug, "ambiguity: " + target.getFiducialId(), target.getPoseAmbiguity());
        }
    }
    
    public Optional<Double> getXMeters() {
        if (hasCorrectTargets()){
            return Optional.of(latestResult.getBestTarget().getBestCameraToTarget().getX());
        }
        return Optional.empty();
    }
    
    public Optional<Double> getYMeters() {
        if (hasCorrectTargets()){
            return Optional.of( latestResult.getBestTarget().getBestCameraToTarget().getY());
        }
        return Optional.empty();
    }
    
    public Optional<Double> getZMeters() {
        if (hasCorrectTargets()){
            return Optional.of( latestResult.getBestTarget().getBestCameraToTarget().getZ());
        }
        return Optional.empty();
    }

    public Optional<Double> getYaw() {
        if (hasCorrectTargets()){
            return Optional.of( latestResult.getBestTarget().getYaw());            
        }
        return Optional.empty();
    }

    public Optional<Double> getPitch() {
        if (hasCorrectTargets()){
            return Optional.of( latestResult.getBestTarget().getPitch());            
        }
        return Optional.empty();
    }

    public Optional<Double> getDistanceToTargetMeters() {
        Optional<Double> x = getXMeters(), y = getYMeters(), z = getZMeters();
        if (x.isEmpty() || y.isEmpty() || z.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(Math.sqrt(Math.pow(x.get(), 2) + Math.pow(y.get(), 2) + Math.pow(z.get(), 2)));
    }

    public Optional<Double> getAngleToTarget() {
        Optional<Double> x = getXMeters(), y = getYMeters(), z = getZMeters();
        if (x.isEmpty() || y.isEmpty() || z.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(Math.toDegrees(Math.atan(getYMeters().get()/getXMeters().get())));
    }

    public boolean hasCorrectTargets() {
        latestResult = m_camera.getLatestResult();
        if (latestResult.hasTargets()){
            if (latestResult.getBestTarget().getFiducialId() == 4 || latestResult.getBestTarget().getFiducialId() == 7) {
                return true;
            }
        }
        return false;
    }
}
