package BobcatLib.Subsystems.Vision.Components;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import BobcatLib.Hardware.Gyros.BaseGyro;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.WpiPoseEstimator;
import BobcatLib.Subsystems.Vision.limelight.Limelight;
import BobcatLib.Subsystems.Vision.limelight.networktables.AngularVelocity3d;
import BobcatLib.Subsystems.Vision.limelight.networktables.LimelightPoseEstimator;
import BobcatLib.Subsystems.Vision.limelight.networktables.LimelightPoseEstimator.BotPose;
import BobcatLib.Subsystems.Vision.limelight.networktables.LimelightSettings.LEDMode;
import BobcatLib.Subsystems.Vision.limelight.networktables.Orientation3d;
import BobcatLib.Subsystems.Vision.limelight.networktables.PoseEstimate;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.Optional;

public class AprilTagDetector implements VisionIO {
  private final Limelight limelight;
  private LimelightPoseEstimator poseEstimator;
  private WpiPoseEstimator wpiPoseEstimator;
  private PoseEstimate poseEstimate;
  private BaseGyro gyro;

  public AprilTagDetector(String limelightName, BaseGyro gyro) {
    this.gyro = gyro;
    limelight = new Limelight(limelightName);
    // Set the limelight to use Pipeline LED control, with the Camera offset of 0,
    // and save.
    limelight
        .getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(Pose3d.kZero)
        .save();
    // Required for megatag2 in periodic() function before fetching pose.
    Rotation3d rot = new Rotation3d(gyro.getYaw());
    limelight
        .getSettings()
        .withRobotOrientation(
            new Orientation3d(
                rot,
                new AngularVelocity3d(
                    DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
        .save();
    try {
      poseEstimate = BotPose.BLUE_MEGATAG2.get(limelight).get();
    } catch (Exception e) {
      poseEstimate = new PoseEstimate(limelight, "botpose_orb_wpiblue", true);
    }
  }

  public void updateInputs(VisionLLIOInputs inputs) {
    inputs.algaeDetected = false;
    inputs.coralDetected = false;
    inputs.pose = getPose().pose.toPose2d();
  }

  public void periodic() {
    // Required for megatag2 in periodic() function before fetching pose.
    Rotation3d rot = new Rotation3d(gyro.getYaw());
    limelight
        .getSettings()
        .withRobotOrientation(
            new Orientation3d(
                rot,
                new AngularVelocity3d(
                    DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
        .save();
  }

  public PoseEstimate getPose() {
    Optional<PoseEstimate> visionEstimate =
        poseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
    visionEstimate.ifPresent(
        (PoseEstimate intPoseEstimate) -> {
          // If the average tag distance is less than 4 meters,
          // there are more than 0 tags in view,
          // and the average ambiguity between tags is less than 30% then we update the pose
          // estimation.
          if (intPoseEstimate.avgTagDist < 4
              && intPoseEstimate.tagCount > 0
              && intPoseEstimate.getMinTagAmbiguity() < 0.3) {
            wpiPoseEstimator.swerveDrivePoseEstimator.addVisionMeasurement(
                poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
          }
        });
    return poseEstimate;
  }
}
