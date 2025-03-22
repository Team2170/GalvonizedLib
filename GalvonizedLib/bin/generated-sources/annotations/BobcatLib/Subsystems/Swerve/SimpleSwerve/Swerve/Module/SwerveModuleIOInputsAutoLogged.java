package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Offset", offset);
    table.put("Velocity", velocity);
    table.put("AbsAngle", absAngle);
    table.put("Angle", angle);
  }

  @Override
  public void fromLog(LogTable table) {
    offset = table.get("Offset", offset);
    velocity = table.get("Velocity", velocity);
    absAngle = table.get("AbsAngle", absAngle);
    angle = table.get("Angle", angle);
  }

  public SwerveModuleIOInputsAutoLogged clone() {
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.offset = this.offset;
    copy.velocity = this.velocity;
    copy.absAngle = this.absAngle;
    copy.angle = this.angle;
    return copy;
  }
}
