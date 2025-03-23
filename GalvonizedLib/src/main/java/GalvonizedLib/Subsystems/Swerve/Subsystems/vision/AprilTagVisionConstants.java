// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package GalvonizedLib.Subsystems.Swerve.Subsystems.vision;

public class AprilTagVisionConstants {

  public class limelightConstants {
    public static final double rotationTolerance = 100;
    public static final double throwoutDist = 4;
    public static final double xySingleTagStdDev = 0.4;
    public static final double thetaSingleTagStdDev = 9999999;
    public static final double xyMultiTagStdDev = 0.3;
    public static final double thetaMultiTagStdDev = 99999999;
    public static final int[] validTags =
        new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
  }
}
