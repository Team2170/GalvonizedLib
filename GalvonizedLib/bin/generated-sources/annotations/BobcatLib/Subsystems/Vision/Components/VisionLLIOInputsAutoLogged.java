package BobcatLib.Subsystems.Vision.Components;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionLLIOInputsAutoLogged extends VisionIO.VisionLLIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("CoralDetected", coralDetected);
    table.put("AlgaeDetected", algaeDetected);
    table.put("Pose", pose);
  }

  @Override
  public void fromLog(LogTable table) {
    coralDetected = table.get("CoralDetected", coralDetected);
    algaeDetected = table.get("AlgaeDetected", algaeDetected);
    pose = table.get("Pose", pose);
  }

  public VisionLLIOInputsAutoLogged clone() {
    VisionLLIOInputsAutoLogged copy = new VisionLLIOInputsAutoLogged();
    copy.coralDetected = this.coralDetected;
    copy.algaeDetected = this.algaeDetected;
    copy.pose = this.pose;
    return copy;
  }
}
