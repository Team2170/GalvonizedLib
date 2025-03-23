package GalvonizedLib.Subsystems.Swerve.Subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("YawPosition", yawPosition);
    table.put("RollPosition", rollPosition);
    table.put("PitchPosition", pitchPosition);
    table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
    table.put("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
    table.put("RollVelocityRadPerSec", rollVelocityRadPerSec);
    table.put("OdometryYawTimestamps", odometryYawTimestamps);
    table.put("OdometryYawPositions", odometryYawPositions);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yawPosition = table.get("YawPosition", yawPosition);
    rollPosition = table.get("RollPosition", rollPosition);
    pitchPosition = table.get("PitchPosition", pitchPosition);
    yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
    pitchVelocityRadPerSec = table.get("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
    rollVelocityRadPerSec = table.get("RollVelocityRadPerSec", rollVelocityRadPerSec);
    odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
    odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.yawPosition = this.yawPosition;
    copy.rollPosition = this.rollPosition;
    copy.pitchPosition = this.pitchPosition;
    copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
    copy.pitchVelocityRadPerSec = this.pitchVelocityRadPerSec;
    copy.rollVelocityRadPerSec = this.rollVelocityRadPerSec;
    copy.odometryYawTimestamps = this.odometryYawTimestamps.clone();
    copy.odometryYawPositions = this.odometryYawPositions.clone();
    return copy;
  }
}
