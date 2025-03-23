package GalvonizedLib.Subsystems.Swerve.Subsystems.vision;

import GalvonizedLib.Subsystems.Swerve.Subsystems.vision.util.VisionObservation.LLTYPE;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class limelightConstants {
  public final String name;
  public final LLTYPE limelightType;
  public final double verticalFOV;
  public final double horizontalFOV;
  public final double limelightMountHeight;
  public final int detectorPiplineIndex;
  public final int apriltagPipelineIndex;
  public final int horPixels;
  public final Vector<N3> visionMeasurementStdDevs;

  /**
   * Limelight Constants to be used when creating swerve modules.
   *
   * @param name
   * @param limelightType
   * @param verticalFOV
   * @param horizontalFOV
   * @param limelightMountHeight
   * @param detectorPiplineIndex
   * @param apriltagPipelineIndex
   * @param horPixels
   * @param visionMeasurementStdDevs
   */
  public limelightConstants(
      String name,
      LLTYPE limelightType,
      double verticalFOV,
      double horizontalFOV,
      double limelightMountHeight,
      int detectorPiplineIndex,
      int apriltagPipelineIndex,
      int horPixels,
      Vector<N3> visionMeasurementStdDevs) {
    this.name = name;
    this.verticalFOV = verticalFOV; // degrees obviously
    this.horizontalFOV = horizontalFOV;
    this.limelightMountHeight = limelightMountHeight;
    this.detectorPiplineIndex = detectorPiplineIndex;
    this.apriltagPipelineIndex = apriltagPipelineIndex;
    this.horPixels = horPixels;
    this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    this.limelightType = limelightType;
  }
}

// enum LLTYPE {
//   LL3,
//   LL3G,
//   LL4
// }
