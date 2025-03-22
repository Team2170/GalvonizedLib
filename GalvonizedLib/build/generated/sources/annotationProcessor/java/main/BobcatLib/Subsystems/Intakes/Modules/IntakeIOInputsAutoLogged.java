package BobcatLib.Subsystems.Intakes.Modules;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeModuleIO.IntakeIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotAngle", pivotAngle);
    table.put("RollerVelocity", rollerVelocity);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotAngle = table.get("PivotAngle", pivotAngle);
    rollerVelocity = table.get("RollerVelocity", rollerVelocity);
  }

  public IntakeIOInputsAutoLogged clone() {
    IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
    copy.pivotAngle = this.pivotAngle;
    copy.rollerVelocity = this.rollerVelocity;
    return copy;
  }
}
