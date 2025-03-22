package BobcatLib.Subsystems.Swerve.SimpleSwerve.Containers;

import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.Utility.LoadablePathPlannerAuto;
import BobcatLib.Subsystems.Vision.Components.AprilTagDetector;
import BobcatLib.Subsystems.Vision.Components.Utility.LimeLightConfig;
import BobcatLib.Subsystems.Vision.Components.VisionIO.target;
import BobcatLib.Subsystems.Vision.VisionSubsystem;
import java.util.List;

public class SwerveWithVision extends SwerveBase {

  public final VisionSubsystem limelightVision;

  public SwerveWithVision(
      OI driver_controller,
      OI secondary_controller,
      List<LoadablePathPlannerAuto> autos,
      String robotName,
      boolean isSim,
      Alliance alliance,
      PIDConstants tranPidPathPlanner,
      PIDConstants rotPidPathPlanner,
      String VisionName,
      List<target> targets,
      LimeLightConfig ll_cfg) {
    super(
        driver_controller,
        secondary_controller,
        autos,
        robotName,
        isSim,
        alliance,
        tranPidPathPlanner,
        rotPidPathPlanner);

    // Initialize the limelight Vision Subystem !
    limelightVision =
        new VisionSubsystem(
            VisionName, new AprilTagDetector("VisionDetector", super.s_Swerve.getBaseGyro()));
  }

  public SwerveWithVision(
      OI driver_controller,
      List<LoadablePathPlannerAuto> autos,
      String robotName,
      boolean isSim,
      Alliance alliance,
      PIDConstants tranPidPathPlanner,
      PIDConstants rotPidPathPlanner,
      String VisionName,
      List<target> targets,
      LimeLightConfig ll_cfg) {
    super(
        driver_controller,
        autos,
        robotName,
        isSim,
        alliance,
        tranPidPathPlanner,
        rotPidPathPlanner);

    // Initialize the limelight Vision Subystem !
    limelightVision =
        new VisionSubsystem(
            VisionName, new AprilTagDetector("VisionDetector", super.s_Swerve.getBaseGyro()));
  }

  public void periodic() {
    limelightVision.periodic();
  }

  public void updateLoadedPaths(List<LoadablePathPlannerAuto> autos) {
    super.updateLoadedPaths(autos);
  }
}
