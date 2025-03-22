package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Hardware.Motors.Utility.CTRE.PidControllerWrapper;
import BobcatLib.Hardware.Motors.Utility.MotorConfigurator;
import BobcatLib.Logging.FaultsAndErrors.FalconFaults;
import BobcatLib.Utilities.CANDeviceDetails;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The FalconMotor class implements the MotorIO interface and provides an abstraction for
 * controlling a Falcon motor. It handles motor initialization, configuration, and control
 * operations such as setting speed, angle, and performing fault checks.
 */
public class FalconMotor implements MotorIO {

  private int motorCanId = 0;
  private final SimpleMotorFeedforward motorFeedFordward;
  private MotorConfigurator motorConfig;
  private TalonFX mMotor;
  private String busName = "";
  private final DutyCycleOut motorDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage motorVelocity = new VelocityVoltage(0);
  private final PositionVoltage motorPositionVoltage = new PositionVoltage(0);
  private final VelocityTorqueCurrentFOC motorVelocityTorqueFOC = new VelocityTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC motorPositionTorqueFOC = new PositionTorqueCurrentFOC(0);
  private FalconFaults faults;
  private CANDeviceDetails details;
  private MotorConfigs config;

  /**
   * Constructs a new FalconMotor instance.
   *
   * @param details The CAN details of the motor.
   * @param busname The CAN bus name.
   * @param config The motor configuration parameters.
   */
  public FalconMotor(CANDeviceDetails details, String busname, MotorConfigs config) {
    this.config = config;
    this.details = details;
    int id = details.getDeviceNumber();
    this.busName = busname;
    motorCanId = id;
    double motorKS = 0.00;
    double motorKV = 0.00;
    double motorKA = 0.00;
    motorFeedFordward = new SimpleMotorFeedforward(motorKS, motorKV, motorKA);
    if (busname == "" || busname == "rio") {
      mMotor = new TalonFX(id);
    } else {
      mMotor = new TalonFX(id, busName);
    }
    configMotor(config);
    faults = new FalconFaults(mMotor, motorCanId);
  }

  /**
   * Configures the motor with the provided parameters.
   *
   * @param cfg The MotorConfigs object containing configuration parameters.
   */
  public void configMotor(MotorConfigs cfg) {
    TalonFXConfiguration internalConfig = motorConfig.getCtreConfig();
    internalConfig.MotorOutput.withInverted(new InvertedWrapper(cfg.isInverted).asCTRE());
    internalConfig.MotorOutput.withNeutralMode(
        new NeutralModeWrapper(cfg.mode).asNeutralModeValue());
    internalConfig.Feedback.withSensorToMechanismRatio(cfg.motorToGearRatio);
    internalConfig.Slot0 =
        new PidControllerWrapper(new Slot0Configs())
            .with_kP(cfg.kP)
            .with_kI(cfg.kI)
            .with_kD(cfg.kD)
            .getSlot0Config();
    internalConfig.CurrentLimits.withStatorCurrentLimit(cfg.optionalCtre.StatorSupplyCurrentLimit);
    internalConfig.CurrentLimits.withStatorCurrentLimitEnable(
        cfg.optionalCtre.StatorSupplyCurrentEnable);
    motorConfig.update(internalConfig);
    mMotor.getConfigurator().apply(motorConfig.getCtreConfig());
  }

  /**
   * Configures the motor with software limit switch settings.
   *
   * @param cfgLimits The {@link SoftwareLimitSwitchConfigs} instance containing the desired
   *     software limit switch configurations.
   * @return The updated {@code FalconMotor} instance for method chaining.
   */
  public FalconMotor withLimits(SoftwareLimitSwitchConfigs cfgLimits) {
    TalonFXConfiguration intConfig = motorConfig.getCtreConfig();
    intConfig.withSoftwareLimitSwitch(cfgLimits);
    motorConfig.update(intConfig);
    mMotor.getConfigurator().apply(motorConfig.getCtreConfig());
    return this;
  }

  /**
   * Updates the motor inputs.
   *
   * @param inputs The MotorIOInputs object to populate with motor data.
   */
  public void updateInputs(MotorIOInputs inputs) {
    inputs.motorPosition = getPosition();
    inputs.motorVelocity = getVelocity();
  }

  /**
   * Retrieves the motor's current position.
   *
   * @return The motor position as a Rotation2d object.
   */
  public Rotation2d getPosition() {
    double pos = mMotor.getPosition().getValueAsDouble();
    return Rotation2d.fromRotations(pos);
  }

  /**
   * Retrieves the motor's current velocity.
   *
   * @return The motor velocity in meters per second.
   */
  public double getVelocity() {
    return mMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Sets the motor's speed.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param isOpenLoop Whether the control is open-loop.
   */
  public void setSpeed(double speedInMPS, boolean isOpenLoop) {
    if (config.optionalCtre.useFOC) {
      double mechanismCircumference = motorConfig.getCtreConfig().Feedback.SensorToMechanismRatio;
      double speedInRPS = speedInMPS / mechanismCircumference;
      setSpeed(speedInRPS, mechanismCircumference, isOpenLoop);
      return;
    }
    motorDutyCycle.Output = speedInMPS;
    mMotor.setControl(motorDutyCycle);
  }

  /**
   * Sets the motor's speed.
   *
   * @param speedInRPS The desired speed in meters per second.
   * @param isOpenLoop Whether the control is open-loop.
   */
  public void setSpeedFOC(double speedInRPS, boolean isOpenLoop) {
    mMotor.setControl(motorVelocityTorqueFOC.withVelocity(speedInRPS));
  }

  /**
   * Sets the motor's speed with additional parameters.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param mechanismCircumference The circumference of the mechanism.
   * @param isOpenLoop Whether the control is open-loop.
   */
  public void setSpeed(double speedInMPS, double mechanismCircumference, boolean isOpenLoop) {
    if (config.optionalCtre.useFOC) {
      double speedInRPS = speedInMPS / mechanismCircumference;
      setSpeed(speedInRPS, mechanismCircumference, isOpenLoop);
      return;
    }
    motorVelocity.Velocity = speedInMPS / mechanismCircumference;
    motorVelocity.FeedForward = motorFeedFordward.calculate(speedInMPS);
    mMotor.setControl(motorVelocity);
  }

  /**
   * Sets the motor's speed with additional parameters. Using FOC
   *
   * @param speedInRPS The desired speed in meters per second.
   * @param mechanismCircumference The circumference of the mechanism.
   * @param isOpenLoop Whether the control is open-loop.
   */
  public void setSpeedFOC(double speedInRPS, double mechanismCircumference, boolean isOpenLoop) {
    double vel = speedInRPS / mechanismCircumference;
    mMotor.setControl(motorVelocityTorqueFOC.withVelocity(vel));
  }

  /**
   * Sets the motor's angle in rotations.
   *
   * @param angleInRotations The desired angle in rotations.
   */
  public void setAngle(double angleInRotations) {
    mMotor.setControl(motorPositionVoltage.withPosition(angleInRotations));
  }

  /**
   * Sets the motor's angle in rotations. Use FOC
   *
   * @param angleInRotations The desired angle in rotations.
   */
  public void setAngleFOC(double angleInRotations) {
    mMotor.setControl(motorPositionTorqueFOC.withPosition(angleInRotations));
  }

  /**
   * Sets the motor control voltage for SysID mode.
   *
   * @param volts The control voltage to apply.
   */
  public void setControl(double volts) {
    VoltageOut sysidControl = new VoltageOut(0);
    sysidControl.withOutput(volts);
    mMotor.setControl(sysidControl);
  }

  /** Stops the motor. */
  public void stopMotor() {
    mMotor.stopMotor();
  }

  /** Checks for motor faults. */
  public void checkForFaults() {
    faults.hasFaultOccured();
  }

  /**
   * Retrieves the TalonFX motor instance.
   *
   * @return The TalonFX motor instance.
   */
  public TalonFX getMotor() {
    return mMotor;
  }

  /**
   * Retrieves the motor's CAN ID.
   *
   * @return The CAN ID of the motor.
   */
  public int getCanId() {
    return motorCanId;
  }

  public MotorConfigurator getConfigurator() {
    return motorConfig;
  }
}
