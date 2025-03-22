package BobcatLib.Subsystems.Vision.Components;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  public class target {
    public boolean isSeen = false;
    public String name = "";
    public double distance;
  }

  @AutoLog
  public class VisionLLIOInputs {
    public boolean coralDetected;
    public boolean algaeDetected;
    public Pose2d pose;
  }

  public default void updateInputs(VisionLLIOInputs inputs) {}

  public default void periodic() {}
}
