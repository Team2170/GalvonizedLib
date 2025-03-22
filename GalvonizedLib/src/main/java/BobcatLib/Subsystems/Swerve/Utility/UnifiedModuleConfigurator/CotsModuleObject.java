package BobcatLib.Subsystems.Swerve.Utility.UnifiedModuleConfigurator;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.SensorDirectionWrapper;
import BobcatLib.Subsystems.Swerve.Utility.CotsModuleSwerveConstants;
import BobcatLib.Subsystems.Swerve.Utility.UnifiedModuleConfigurator.LoadedConfig.ConfigFileJson;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class CotsModuleObject {
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
  public InvertedWrapper driveMotorInvert;
  /** angleMotorInvert */
  public InvertedWrapper angleMotorInvert;
  /** absoluteEncoderInvert */
  public SensorDirectionWrapper absoluteEncoderInvert;

  public CotsModuleObject() {}

  public CotsModuleObject(
      double wheelDiameter,
      double angleGearRatio,
      double driveGearRatio,
      double angleKP,
      double angleKI,
      double angleKD,
      InvertedWrapper driveMotorInvert,
      InvertedWrapper angleMotorInvert,
      SensorDirectionWrapper cancoderInvert) {
    this.wheelDiameter = wheelDiameter * Math.PI;
    this.wheelCircumference = wheelDiameter;
    this.angleGearRatio = angleGearRatio;
    this.driveGearRatio = driveGearRatio;
    this.angleKP = angleKP;
    this.angleKI = angleKI;
    this.angleKD = angleKD;
    this.driveMotorInvert = driveMotorInvert;
    this.angleMotorInvert = angleMotorInvert;
    this.absoluteEncoderInvert = cancoderInvert;
  }

  private ConfigFileJson getMotorConfig(LoadedConfig cfg, String motorType) throws Exception {
    Map<String, ConfigFileJson> motorConfigMap =
        Map.of(
            "Falcon500", cfg.Falcon500,
            "KrakenX60", cfg.KrakenX60,
            "Neo", cfg.Neo,
            "Vortex", cfg.Vortex);

    ConfigFileJson motorConfig = motorConfigMap.get(motorType);
    if (motorConfig == null) {
      throw new Exception("Invalid motor type: " + motorType);
    }
    return motorConfig;
  }

  public CotsModuleObject withConfiguration(
      String manufacturer, String moduleType, String MotorType, String level) throws Exception {
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve/modules");
    assert directory.exists();
    File manufacturer_directory = new File(directory, manufacturer);
    assert manufacturer_directory.exists();
    String name = moduleType + ".json";
    File moduleFile = new File(manufacturer_directory, name);
    assert new File(manufacturer_directory, name).exists();
    LoadedConfig cfg = new LoadedConfig();
    CotsModuleObject cots;
    cfg = new ObjectMapper().readValue(moduleFile, LoadedConfig.class);
    driveGearRatio = 0.00;

    Map<String, Double> gearRatioMap = new HashMap<>();
    gearRatioMap.put("L1", cfg.driveGearRatios.L1);
    gearRatioMap.put("L2", cfg.driveGearRatios.L2);
    gearRatioMap.put("L3", cfg.driveGearRatios.L3);
    gearRatioMap.put("L4", cfg.driveGearRatios.L4);
    gearRatioMap.put("X1_10", cfg.driveGearRatios.X1_10);
    gearRatioMap.put("X1_11", cfg.driveGearRatios.X1_11);
    gearRatioMap.put("X1_12", cfg.driveGearRatios.X1_12);
    gearRatioMap.put("X2_10", cfg.driveGearRatios.X2_10);
    gearRatioMap.put("X2_11", cfg.driveGearRatios.X2_11);
    gearRatioMap.put("X2_12", cfg.driveGearRatios.X2_12);
    gearRatioMap.put("X3_10", cfg.driveGearRatios.X3_12);
    gearRatioMap.put("X3_11", cfg.driveGearRatios.X3_13);
    gearRatioMap.put("X3_12", cfg.driveGearRatios.X3_14);
    driveGearRatio = gearRatioMap.getOrDefault(level, 0.0);

    ConfigFileJson motor_cfg = getMotorConfig(cfg, MotorType);
    cots =
        new CotsModuleObject(
            motor_cfg.wheelDiameter,
            motor_cfg.angleGearRatio,
            driveGearRatio,
            motor_cfg.angleKP,
            motor_cfg.angleKI,
            motor_cfg.angleKD,
            new InvertedWrapper(motor_cfg.driveMotorInvert),
            new InvertedWrapper(motor_cfg.angleMotorInvert),
            new SensorDirectionWrapper(motor_cfg.absoluteEncoderInvert));
    return cots;
  }

  public CotsModuleSwerveConstants to() {
    return new CotsModuleSwerveConstants(
        wheelDiameter,
        angleGearRatio,
        driveGearRatio,
        angleKP,
        angleKI,
        angleKD,
        driveMotorInvert,
        angleMotorInvert,
        absoluteEncoderInvert);
  }
}
