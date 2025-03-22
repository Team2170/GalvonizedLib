package BobcatLib.Subsystems.Elevators.Modules;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorModuleIO.ElevatorIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ElevatorPosition", elevatorPosition);
    table.put("CurrentSetPoint", currentSetPoint);
  }

  @Override
  public void fromLog(LogTable table) {
    elevatorPosition = table.get("ElevatorPosition", elevatorPosition);
    currentSetPoint = table.get("CurrentSetPoint", currentSetPoint);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.elevatorPosition = this.elevatorPosition;
    copy.currentSetPoint = this.currentSetPoint;
    return copy;
  }
}
