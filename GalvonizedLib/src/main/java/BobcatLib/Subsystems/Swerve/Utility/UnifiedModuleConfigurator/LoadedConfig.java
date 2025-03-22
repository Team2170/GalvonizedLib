package BobcatLib.Subsystems.Swerve.Utility.UnifiedModuleConfigurator;

public class LoadedConfig {
  public class ConfigFileJson {
    /** wheelDiameter */
    public double wheelDiameter;
    /** wheelCircumference */
    public double wheelCircumference;
    /** driveGearRatio */
    public double angleGearRatio;
    /** wheelDiameter */
    public double driveGearRatio;
    /** angleKP */
    public double angleKP;
    /** angleKI */
    public double angleKI;
    /** angleKD */
    public double angleKD;
    /** driveMotorInvert */
    public boolean driveMotorInvert;
    /** angleMotorInvert */
    public boolean angleMotorInvert;
    /** absoluteEncoderInvert */
    public boolean absoluteEncoderInvert;
  }

  public class levels {
    public double X1_10;
    public double X1_11;
    public double X1_12;
    public double X2_10;
    public double X2_11;
    public double X2_12;
    public double X3_12;
    public double X3_13;
    public double X3_14;
    public double L1;
    public double L2;
    public double L3;
    public double L4;
  }

  public ConfigFileJson Falcon500;
  public ConfigFileJson KrakenX60;
  public ConfigFileJson Neo;
  public ConfigFileJson Vortex;
  public levels driveGearRatios;
}
