package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Gyro;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("YawPosition", yawPosition);
    table.put("PitchPosition", pitchPosition);
    table.put("RollPosition", rollPosition);
    table.put("OdometryYawTimestamps", odometryYawTimestamps);
    table.put("OdometryYawPositions", odometryYawPositions);
    table.put("ZAngularVelocityDegPerSec", zAngularVelocityDegPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yawPosition = table.get("YawPosition", yawPosition);
    pitchPosition = table.get("PitchPosition", pitchPosition);
    rollPosition = table.get("RollPosition", rollPosition);
    odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
    odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
    zAngularVelocityDegPerSec = table.get("ZAngularVelocityDegPerSec", zAngularVelocityDegPerSec);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.yawPosition = this.yawPosition;
    copy.pitchPosition = this.pitchPosition;
    copy.rollPosition = this.rollPosition;
    copy.odometryYawTimestamps = this.odometryYawTimestamps.clone();
    copy.odometryYawPositions = this.odometryYawPositions.clone();
    copy.zAngularVelocityDegPerSec = this.zAngularVelocityDegPerSec;
    return copy;
  }
}
