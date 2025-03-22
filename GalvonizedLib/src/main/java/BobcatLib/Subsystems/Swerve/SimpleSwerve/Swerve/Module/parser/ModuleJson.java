package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser;

public class ModuleJson {
  public ModuleMotorJson drive;
  public ModuleMotorJson angle;
  public ModuleEncoderJson encoder;
  public String type = "";
  public String manufacturer = "";
  public String level = "";
  public double wheelCircumference = 0.0;
  /** Drive Current Limit */
  public int driveSupplyCurrentLimit = 35;

  public int driveStatorCurrentLimit = 100;
  /** Drive ENable Current Limit */
  public boolean driveEnableSupplyCurrentLimit = false;

  public boolean driveEnableStatorCurrentLimit = true;
  /** Angle Current Limit */
  public int angleSupplyCurrentLimit = 25;

  public int angleStatorCurrentLimit = 25;
  /** ANgle Enable Current Limit */
  public boolean angleEnableSupplyCurrentLimit = false;

  public boolean angleEnableStatorCurrentLimit = true;
  /** Ioen Loop Ramp */
  public double openLoopRamp = 0.25;
  /** Closed Loop Ramp */
  public double closedLoopRamp = 0.0;
}
