package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Offset", offset);
    table.put("DrivePositionRot", drivePositionRot);
    table.put("DriveVelocityRotPerSec", driveVelocityRotPerSec);
    table.put("DriveAccelerationRadPerSecSquared", driveAccelerationRadPerSecSquared);
    table.put("CanCoderPositionRot", canCoderPositionRot);
    table.put("CanCoderPositionDeg", canCoderPositionDeg);
    table.put("TurnAngularVelocityRadPerSec", turnAngularVelocityRadPerSec);
    table.put("InternalTempDrive", internalTempDrive);
    table.put("ProcessorTempDrive", processorTempDrive);
    table.put("InternalTempAngle", internalTempAngle);
    table.put("ProcessorTempAngle", processorTempAngle);
    table.put("AppliedDriveVoltage", appliedDriveVoltage);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("AngleCurrentAmps", angleCurrentAmps);
    table.put("AngleAppliedVoltage", angleAppliedVoltage);
    table.put("OdometryTimestamps", odometryTimestamps);
    table.put("OdometryDrivePositionsRad", odometryDrivePositionsRad);
    table.put("OdometryAnglePositions", odometryAnglePositions);
  }

  @Override
  public void fromLog(LogTable table) {
    offset = table.get("Offset", offset);
    drivePositionRot = table.get("DrivePositionRot", drivePositionRot);
    driveVelocityRotPerSec = table.get("DriveVelocityRotPerSec", driveVelocityRotPerSec);
    driveAccelerationRadPerSecSquared =
        table.get("DriveAccelerationRadPerSecSquared", driveAccelerationRadPerSecSquared);
    canCoderPositionRot = table.get("CanCoderPositionRot", canCoderPositionRot);
    canCoderPositionDeg = table.get("CanCoderPositionDeg", canCoderPositionDeg);
    turnAngularVelocityRadPerSec =
        table.get("TurnAngularVelocityRadPerSec", turnAngularVelocityRadPerSec);
    internalTempDrive = table.get("InternalTempDrive", internalTempDrive);
    processorTempDrive = table.get("ProcessorTempDrive", processorTempDrive);
    internalTempAngle = table.get("InternalTempAngle", internalTempAngle);
    processorTempAngle = table.get("ProcessorTempAngle", processorTempAngle);
    appliedDriveVoltage = table.get("AppliedDriveVoltage", appliedDriveVoltage);
    driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
    angleCurrentAmps = table.get("AngleCurrentAmps", angleCurrentAmps);
    angleAppliedVoltage = table.get("AngleAppliedVoltage", angleAppliedVoltage);
    odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
    odometryDrivePositionsRad = table.get("OdometryDrivePositionsRad", odometryDrivePositionsRad);
    odometryAnglePositions = table.get("OdometryAnglePositions", odometryAnglePositions);
  }

  public SwerveModuleIOInputsAutoLogged clone() {
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.offset = this.offset;
    copy.drivePositionRot = this.drivePositionRot;
    copy.driveVelocityRotPerSec = this.driveVelocityRotPerSec;
    copy.driveAccelerationRadPerSecSquared = this.driveAccelerationRadPerSecSquared;
    copy.canCoderPositionRot = this.canCoderPositionRot;
    copy.canCoderPositionDeg = this.canCoderPositionDeg;
    copy.turnAngularVelocityRadPerSec = this.turnAngularVelocityRadPerSec;
    copy.internalTempDrive = this.internalTempDrive;
    copy.processorTempDrive = this.processorTempDrive;
    copy.internalTempAngle = this.internalTempAngle;
    copy.processorTempAngle = this.processorTempAngle;
    copy.appliedDriveVoltage = this.appliedDriveVoltage;
    copy.driveCurrentAmps = this.driveCurrentAmps;
    copy.angleCurrentAmps = this.angleCurrentAmps;
    copy.angleAppliedVoltage = this.angleAppliedVoltage;
    copy.odometryTimestamps = this.odometryTimestamps.clone();
    copy.odometryDrivePositionsRad = this.odometryDrivePositionsRad.clone();
    copy.odometryAnglePositions = this.odometryAnglePositions.clone();
    return copy;
  }
}
