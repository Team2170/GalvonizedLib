package BobcatLib.Subsystems.Climbers.Modules;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ClimberMotorPosition", climberMotorPosition);
  }

  @Override
  public void fromLog(LogTable table) {
    climberMotorPosition = table.get("ClimberMotorPosition", climberMotorPosition);
  }

  public ClimberIOInputsAutoLogged clone() {
    ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
    copy.climberMotorPosition = this.climberMotorPosition;
    return copy;
  }
}
