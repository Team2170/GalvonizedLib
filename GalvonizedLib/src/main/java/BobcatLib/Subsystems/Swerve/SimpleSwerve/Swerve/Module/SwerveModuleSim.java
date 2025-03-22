package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module;

import BobcatLib.Hardware.Encoders.BaseEncoder;
import BobcatLib.Hardware.Encoders.CanCoderWrapper;
import BobcatLib.Hardware.Encoders.EncoderSim;
import BobcatLib.Hardware.Encoders.ThriftyAbsoluteEncoder;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor.DriveWrapper;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor.FalconDriveMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor.KrakenDriveMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor.NeoDriveMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor.SimDriveMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor.VortexDriveMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor.FalconSteerMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor.KrakenSteerMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor.Neo550SteerMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor.NeoSteerMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor.SimSteerMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor.SteerWrapper;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor.VortexSteerMotor;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.math.Conversions;
import BobcatLib.Subsystems.Swerve.Utility.CotsModuleSwerveConstants;
import BobcatLib.Subsystems.Swerve.Utility.UnifiedModuleConfigurator.CotsModuleObject;
import BobcatLib.Utilities.CANDeviceDetails;
import BobcatLib.Utilities.CANDeviceDetails.Manufacturer;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.StringSubscriber;

public class SwerveModuleSim implements SwerveModuleIO {
  public int moduleNumber;
  private Rotation2d angleOffset;
  private SteerWrapper mAngleMotor;
  private DriveWrapper mDriveMotor;
  private BaseEncoder angleAbsEncoder;
  /** NT3 Raw Absolute Angle publisher for the absolute encoder. */
  private final String rawAbsoluteAngleName;
  /** NT3 raw angle motor. */
  private final String rawAngleName;
  /** NT3 Raw drive motor. */
  private final String rawDriveName;

  /* SysId Voltage Control */
  private VoltageOut sysidControl = new VoltageOut(0);

  // Subscribed too the
  public StringSubscriber configSubscriber;
  public String lastConfig = "{}";
  public ModuleJson jsonModule;
  private SwerveModuleState desiredState = new SwerveModuleState();
  public ModuleConstants chosenModule;
  public ModuleLimitsJson swerveLimits;
  public String robotName;

  /**
   * initializes a swerve module with the index and the constants as configured.
   *
   * @param moduleNumber
   */
  public SwerveModuleSim(int moduleNumber, ModuleLimitsJson limits, String robotName) {
    this.moduleNumber = moduleNumber;
    this.swerveLimits = limits;
    this.robotName = robotName;
    rawAbsoluteAngleName = "Module[" + moduleNumber + "] Raw Absolute Encoder";
    rawAngleName = "Module[" + moduleNumber + "] Raw Angle Encoder";
    rawDriveName = "Module[" + moduleNumber + "] Raw Drive Encoder";
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.offset = angleOffset.getRotations();
    inputs.absAngle = getCANcoder().getRotations();

    SwerveModuleState state = getState();
    inputs.velocity = state.speedMetersPerSecond;
    inputs.angle = state.angle.getRotations();
  }

  public void configModule() throws Exception {
    this.angleOffset = Rotation2d.fromRotations(jsonModule.encoder.offset);

    CotsModuleSwerveConstants cotsModule =
        new CotsModuleObject()
            .withConfiguration(
                jsonModule.manufacturer,
                jsonModule.type,
                jsonModule.drive.motor_type,
                jsonModule.level)
            .to();

    chosenModule = new ModuleConstants(cotsModule, jsonModule);

    /* Angle Encoder Config */
    assignAbsEncoder("cancoder", jsonModule.encoder.id);

    /* Angle Motor Config */
    assignSteerMotor(jsonModule.angle.motor_type, jsonModule.angle.id);
    resetToAbsolute();

    /* Drive Motor Config */
    assignDriveMotor(jsonModule.drive.motor_type, jsonModule.drive.id);
  }

  public void assignAbsEncoder(String type, int canId) {
    CANDeviceDetails details;
    switch (type) {
      case "cancoder":
        details = new CANDeviceDetails(canId, Manufacturer.Ctre, "Swerve/Module" + moduleNumber);
        angleAbsEncoder =
            new BaseEncoder(
                new CanCoderWrapper(
                    details, chosenModule.encoderConstants, jsonModule.encoder.canbus));
        break;
      case "thrifty":
        details = new CANDeviceDetails(canId, Manufacturer.Thrifty, "Swerve/Module" + moduleNumber);
        angleAbsEncoder =
            new BaseEncoder(new ThriftyAbsoluteEncoder(details, chosenModule.encoderConstants));
        break;
      default:
        details = new CANDeviceDetails(canId, Manufacturer.Ctre, "Swerve/Module" + moduleNumber);
        angleAbsEncoder =
            new BaseEncoder(
                new EncoderSim(details, chosenModule.encoderConstants, jsonModule.encoder.canbus));
    }
  }

  public void assignSteerMotor(String type, int canId) {
    CANDeviceDetails details;
    switch (type) {
      case "KrakenX60":
        details = new CANDeviceDetails(canId, Manufacturer.Ctre, "Swerve/Module" + moduleNumber);
        mAngleMotor =
            new KrakenSteerMotor(
                details, canId, chosenModule, jsonModule.angle.canbus, swerveLimits);
        break;
      case "Falcon500":
        details = new CANDeviceDetails(canId, Manufacturer.Ctre, "Swerve/Module" + moduleNumber);
        mAngleMotor =
            new FalconSteerMotor(
                details, canId, chosenModule, jsonModule.angle.canbus, swerveLimits);
        break;
      case "Neo550":
        details = new CANDeviceDetails(canId, Manufacturer.Rev, "Swerve/Module" + moduleNumber);
        mAngleMotor = new Neo550SteerMotor(details, canId, chosenModule, swerveLimits);
        break;
      case "Neo":
        details = new CANDeviceDetails(canId, Manufacturer.Rev, "Swerve/Module" + moduleNumber);
        mAngleMotor = new NeoSteerMotor(details, canId, chosenModule, swerveLimits);
        break;
      case "Vortex":
        details = new CANDeviceDetails(canId, Manufacturer.Rev, "Swerve/Module" + moduleNumber);
        mAngleMotor = new VortexSteerMotor(details, canId, chosenModule, swerveLimits);
        break;
      default:
        details = new CANDeviceDetails(canId, Manufacturer.Ctre, "Swerve/Module" + moduleNumber);
        mAngleMotor = new SimSteerMotor(chosenModule, swerveLimits);
    }
    mAngleMotor.withStatorCurrent();
  }

  public void assignDriveMotor(String type, int canId) {
    CANDeviceDetails details;
    switch (type) {
      case "KrakenX60":
        details = new CANDeviceDetails(canId, Manufacturer.Ctre, "Swerve/Module" + moduleNumber);
        mDriveMotor =
            new KrakenDriveMotor(
                details, canId, chosenModule, jsonModule.drive.canbus, swerveLimits);
        break;
      case "Falcon500":
        details = new CANDeviceDetails(canId, Manufacturer.Ctre, "Swerve/Module" + moduleNumber);
        mDriveMotor =
            new FalconDriveMotor(
                details, canId, chosenModule, jsonModule.drive.canbus, swerveLimits);
        break;
      case "Neo":
        details = new CANDeviceDetails(canId, Manufacturer.Rev, "Swerve/Module" + moduleNumber);
        mDriveMotor = new NeoDriveMotor(details, canId, chosenModule, swerveLimits);
        break;
      case "Vortex":
        details = new CANDeviceDetails(canId, Manufacturer.Rev, "Swerve/Module" + moduleNumber);
        mDriveMotor = new VortexDriveMotor(details, canId, chosenModule, swerveLimits);
        break;
      default:
        details = new CANDeviceDetails(canId, Manufacturer.Rev, "Swerve/Module" + moduleNumber);
        mDriveMotor = new SimDriveMotor(chosenModule, swerveLimits);
    }
    mDriveMotor.withStatorCurrent();
  }

  /**
   * sets the desired state of the module's angle and drive motors.
   *
   * @param currentState
   * @param isOpenLoop
   */
  public void setDesiredState(SwerveModuleState currentState, boolean isOpenLoop) {
    @SuppressWarnings("deprecation")
    SwerveModuleState optimizedState = SwerveModuleState.optimize(currentState, getState().angle);
    setAngle(optimizedState);
    setSpeed(optimizedState, isOpenLoop);

    this.desiredState = optimizedState;
  }

  /**
   * sets the drive motor's speed given the desired state's velocity in rotations/second
   *
   * @param state
   * @param isOpenLoop
   */
  private void setSpeed(SwerveModuleState state, boolean isOpenLoop) {
    mDriveMotor.setSpeed(state, isOpenLoop);
  }

  /**
   * sets the angle motor's position from desired rotations
   *
   * @param state
   */
  private void setAngle(SwerveModuleState state) {
    mAngleMotor.setAngle(state.angle.getRotations());
  }

  /**
   * get the candcoder's absolute position raw in rotation2d from rotations
   *
   * @return cancoder position
   */
  private Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleAbsEncoder.getAbsolutePosition());
  }

  /**
   * gets the absolute encoder position factoring in angle offsett.
   *
   * @return absolute encoder position in rotation2d
   */
  private Rotation2d getAbsCANcoder() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    return Rotation2d.fromRotations(absolutePosition);
  }

  /** Reset the absolute encoder's position in rotations to the internal angle motor position. */
  public void resetToAbsolute() {
    double absolutePosition = getAbsCANcoder().getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  /**
   * gets the current modules state in velocity and position given drive motor velocity in rotations
   * / sec given angle motor position in rotations
   *
   * @return swerve module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getVelocityMetersPerSec(), Rotation2d.fromRotations(mAngleMotor.getPosition()));
  }

  /**
   * gets the position of the modules given motor position (drive ), and the angle motor's position
   *
   * @return swerve module position
   */
  public SwerveModulePosition getPosition() {
    double position = mAngleMotor.getPosition();
    return new SwerveModulePosition(getPositionMeters(), Rotation2d.fromRotations(position));
  }

  /**
   * Gets the current position of the drive motor in meters
   *
   * @return drive motor position, in meters
   */
  public double getPositionMeters() {
    double wheelCircumference = jsonModule.wheelCircumference;
    return Conversions.rotationsToMeters(mDriveMotor.getPosition(), wheelCircumference);
  }

  /* Sys ID component Module Stuff */
  public void runCharachterization(double volts) {
    sysidControl.withOutput(volts);
    mAngleMotor.setPosition(0);
    mDriveMotor.setControl(volts);
  }

  /**
   * Gets the current velocity of the drive motor in meters per second
   *
   * @return velocity, in meter per second
   */
  public double getVelocityMetersPerSec() {
    double wheelCircumference = jsonModule.wheelCircumference;
    return Conversions.RPSToMPS(mDriveMotor.getVelocity(), wheelCircumference);
  }

  /**
   * @return motor Voltage of the given motor
   */
  public double getVoltage() {
    return mDriveMotor.getMotorVoltage();
  }

  /** Stops the motors properly. */
  public void stopMotors() {
    mDriveMotor.stopMotor();
    mAngleMotor.stopMotor();
  }

  /** Gets the Module Number/Index */
  public int getModuleNumber() {
    return moduleNumber;
  }

  /** Gets the Desired State of the module */
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }
}
