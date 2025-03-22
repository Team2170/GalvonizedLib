package BobcatLib.Hardware.Sensors.SpatialSensor;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SpatialIOInputsAutoLogged extends SpatialIO.SpatialIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Front_left_distance", front_left_distance);
    table.put("Front_right_distance", front_right_distance);
    table.put("IsAligned", isAligned);
  }

  @Override
  public void fromLog(LogTable table) {
    front_left_distance = table.get("Front_left_distance", front_left_distance);
    front_right_distance = table.get("Front_right_distance", front_right_distance);
    isAligned = table.get("IsAligned", isAligned);
  }

  public SpatialIOInputsAutoLogged clone() {
    SpatialIOInputsAutoLogged copy = new SpatialIOInputsAutoLogged();
    copy.front_left_distance = this.front_left_distance;
    copy.front_right_distance = this.front_right_distance;
    copy.isAligned = this.isAligned;
    return copy;
  }
}
