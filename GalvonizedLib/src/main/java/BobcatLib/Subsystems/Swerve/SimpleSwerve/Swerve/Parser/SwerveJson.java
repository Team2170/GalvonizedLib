package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;

public class SwerveJson {
  public BaseJson base = new BaseJson();
  public ModuleLimitsJson moduleSpeedLimits = new ModuleLimitsJson();
  public chassisLimitsJson chassisSpeedLimits = new chassisLimitsJson();
  public drivePIDJson rotationPID = new drivePIDJson();
  public drivePIDJson autoAlignPID = new drivePIDJson();
  public drivePIDJson translationPID = new drivePIDJson();
  public SwerveIndicatorJson indicator = new SwerveIndicatorJson();
  public driveCharacteristicsJson driveCharacteristics = new driveCharacteristicsJson();
}
