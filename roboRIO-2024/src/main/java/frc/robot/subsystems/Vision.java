package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.photonvision.PhotonCamera;
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

    LEDs m_leds;
    public Vision(LEDs leds) {
        m_leds = leds;
    }

    PhotonCamera m_camera = new PhotonCamera("Shelldon");

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


    public Optional<Double> getAngleToTarget() {

        var result = m_camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();

        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == 4 || target.getFiducialId() == 7) {
                // if (target.getPoseAmbiguity() < 0.1) {
                    DashboardHelper.putNumber(LogLevel.Debug, "yaw", target.getYaw());
                    return Optional.of(target.getPitch());
                // }
            }
            DashboardHelper.putBoolean(LogLevel.Debug, "tag detected", false);
            return Optional.empty();
        }

        return Optional.empty(); 

    }/*double angle = Math.atan(Units.Inches.toBaseUnits(target.getBestCameraToTarget().getY())
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

    public Optional<Double> getDistanceToTarget() {
        var result = m_camera.getLatestResult();
        Optional<Double> pitch = getAngleToTarget();
        List<PhotonTrackedTarget> targets = result.getTargets();
        double distance;
        if(!targets.isEmpty() && pitch.isPresent()) {
            distance = 63 / Math.toDegrees(Math.tan(Math.toRadians(pitch.get())));

            return Optional.of(distance);
        } 
        return Optional.empty();
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
            DashboardHelper.putString(LogLevel.Info, "angle to speaker", getAngleToSpeaker().toString());
            DashboardHelper.putString(LogLevel.Info, "distance to target", getDistanceToTarget().toString());
        }

        DashboardHelper.putBoolean(LogLevel.Debug, "tag detected", !noTarget());

        for (PhotonTrackedTarget target : targets) {
            DashboardHelper.putNumber(LogLevel.Debug, "ambiguity: " + target.getFiducialId(), target.getPoseAmbiguity());
        }
    }
    
    //TODO: yk
    // public Optional<Double> getXMeters() {
    //     return m_camera.getLatestResult().getTargets().
    // }
}
