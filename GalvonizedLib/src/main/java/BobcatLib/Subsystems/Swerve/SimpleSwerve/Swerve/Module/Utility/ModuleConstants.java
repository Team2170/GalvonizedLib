package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility;

import BobcatLib.Hardware.Encoders.EncoderConstants;
import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.SensorDirectionWrapper;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleJson;
import BobcatLib.Subsystems.Swerve.Utility.CotsModuleSwerveConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Module Constants used for converting the json configs to the COTS SwerveConstants */
public class ModuleConstants {
  public double wheelCircumference;
  private CotsModuleSwerveConstants chosenModule;
  public double driveGearRatio = 0.0;
  public double angleGearRatio = 0.0;
  public NeutralModeWrapper angleNeutralMode;
  public NeutralModeWrapper driveNeutralMode;
  public InvertedWrapper angleMotorInvert;
  public InvertedWrapper driveMotorInvert;
  public EncoderConstants encoderConstants;
  public SensorDirectionWrapper absoluteEncoderInvert;
  public PIDConstants anglePID;
  public PIDConstants drivePID;
  public double driveConversionPositionFactor;
  public double driveConversionVelocityFactor;
  public double angleConversionFactor;
  public ModuleJson json;

  public ModuleConstants(CotsModuleSwerveConstants chosenModule, ModuleJson json) {
    this.chosenModule = chosenModule;
    this.json = json;
    wheelCircumference = chosenModule.wheelCircumference;
    double wheelDiameter = wheelCircumference / Math.PI;
    /* Module Gear Ratios */
    driveGearRatio = chosenModule.driveGearRatio;
    angleGearRatio = chosenModule.angleGearRatio;
    /* Drive & Angle Motor Conversion Factors */
    driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    angleConversionFactor = 360.0 / angleGearRatio;
    /* Motor Inverts */
    angleMotorInvert = chosenModule.angleMotorInvert;
    driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    absoluteEncoderInvert = chosenModule.absoluteEncoderInvert;

    /* Angle Motor PID Values */
    anglePID = new PIDConstants(chosenModule.angleKP, chosenModule.angleKI, chosenModule.angleKD);

    /* Drive Motor PID Values */
    drivePID = new PIDConstants(0.05, 0.00, 0.0);
    double driveKF = 0.0;

    /* Neutral Modes */
    angleNeutralMode = new NeutralModeWrapper(NeutralModeValue.Coast);
    driveNeutralMode = new NeutralModeWrapper(NeutralModeValue.Brake);

    encoderConstants = new EncoderConstants();
    encoderConstants.absoluteEncoderInvert = absoluteEncoderInvert;
  }

  public CotsModuleSwerveConstants getConstants() {
    return chosenModule;
  }
}
