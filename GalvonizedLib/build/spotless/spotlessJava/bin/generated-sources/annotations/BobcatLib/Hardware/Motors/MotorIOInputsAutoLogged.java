package BobcatLib.Hardware.Motors;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MotorIOInputsAutoLogged extends MotorIO.MotorIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorPosition", motorPosition);
    table.put("MotorVelocity", motorVelocity);
    table.put("Faulted", faulted);
  }

  @Override
  public void fromLog(LogTable table) {
    motorPosition = table.get("MotorPosition", motorPosition);
    motorVelocity = table.get("MotorVelocity", motorVelocity);
    faulted = table.get("Faulted", faulted);
  }

  public MotorIOInputsAutoLogged clone() {
    MotorIOInputsAutoLogged copy = new MotorIOInputsAutoLogged();
    copy.motorPosition = this.motorPosition;
    copy.motorVelocity = this.motorVelocity;
    copy.faulted = this.faulted;
    return copy;
  }
}
