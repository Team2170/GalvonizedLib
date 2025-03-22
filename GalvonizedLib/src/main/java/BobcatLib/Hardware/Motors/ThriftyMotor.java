package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Logging.FaultsAndErrors.SparkMaxFaults;
import BobcatLib.Utilities.CANDeviceDetails;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.thethriftybot.Conversion;
import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.Conversion.VelocityUnit;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The ThriftyMotor class implements the MotorIO interface and provides an abstraction for
 * controlling a Neo Nova motor controller. It handles motor initialization, configuration, and
 * control operations such as setting speed, angle, and performing fault checks.
 */
public class ThriftyMotor implements MotorIO {
  private int motorCanId = 0;
  private final SimpleMotorFeedforward motorFeedFordward;
  private ThriftyNova mMotor;
  private String busName = "";
  private SparkMaxFaults faults;
  private CANDeviceDetails details;
  /** Position conversion object for the motor encoder */
  private Conversion positionConversion;
  /** Velocity conversion object for the motor encoder */
  private Conversion velocityConversion;
  /** The position conversion factor for the encoder */
  private double positionConversionFactor = 1.0;
  /** The position conversion factor for the encoder */
  private double velocityConversionFactor = 1.0 / 60.0;

  /**
   * Constructs a NeoMotor instance with the specified CAN ID, bus name, and configuration.
   *
   * @param details The CAN details of the motor.
   * @param busname The bus name where the motor is connected.
   * @param config The motor configuration settings.
   */
  public ThriftyMotor(CANDeviceDetails details, String busname, MotorConfigs config) {
    this.details = details;
    int id = details.getDeviceNumber();
    this.busName = busname;
    motorCanId = id;
    double motorKS = 0.00;
    double motorKV = 0.00;
    double motorKA = 0.00;
    motorFeedFordward = new SimpleMotorFeedforward(motorKS, motorKV, motorKA);
    /* Drive Motor Config */
    mMotor = new ThriftyNova(id);
    configMotor(config);
    mMotor.setEncoderPosition(0);
    faults = new SparkMaxFaults(motorCanId);
  }

  /**
   * Configures the motor with software limit switch settings.
   *
   * @param cfgLimits The {@link LimitSwitchConfig} instance containing the desired limit switch
   *     configurations.
   * @return The updated {@code NeoMotor} instance for method chaining.
   */
  public ThriftyMotor withLimits(LimitSwitchConfig cfgLimits) {
    return this;
  }

  /**
   * Configures the motor settings based on the provided MotorConfigs.
   *
   * @param cfg The MotorConfigs object containing configuration parameters.
   */
  public void configMotor(MotorConfigs cfg) {

    /* Motor Inverts and Neutral Mode */
    mMotor.setInverted(new InvertedWrapper(cfg.isInverted).asThrifty());
    IdleMode motorMode = new NeutralModeWrapper(cfg.mode).asIdleMode();
    if (motorMode.value == 0) {
      mMotor.setBrakeMode(false);
    } else {
      mMotor.setBrakeMode(true);
    }
    /* Current Limiting */
    mMotor.setMaxCurrent(CurrentType.SUPPLY, cfg.optionalRev.SupplyCurrentLimit);
    mMotor.pid0.setP(cfg.kP).setI(cfg.kI).setD(cfg.kD);
    mMotor.pid1.setP(0).setI(0).setD(0).setFF(0.0);
    /* Open and Closed Loop Ramping */
    mMotor.setMaxOutput(1.0);
    mMotor.setRampDown(100);
    mMotor.setRampUp(100);

    positionConversionFactor = cfg.optionalRev.driveConversionVelocityFactor;
    velocityConversionFactor = cfg.optionalRev.driveConversionPositionFactor;

    mMotor.useEncoderType(EncoderType.INTERNAL);
  }

  public void determineConversions(boolean isVelocity) {
    if (isVelocity) {
      positionConversion = new Conversion(PositionUnit.ROTATIONS, EncoderType.INTERNAL);
      velocityConversion = new Conversion(VelocityUnit.ROTATIONS_PER_SEC, EncoderType.INTERNAL);
    } else {
      positionConversion = new Conversion(PositionUnit.DEGREES, EncoderType.INTERNAL);
      velocityConversion = new Conversion(VelocityUnit.DEGREES_PER_SEC, EncoderType.INTERNAL);
    }
  }

  /**
   * Updates the inputs for the motor with the latest sensor data.
   *
   * @param inputs The MotorIOInputs object to update.
   */
  public void updateInputs(MotorIOInputs inputs) {
    inputs.motorPosition = getPosition();
    inputs.motorVelocity = getVelocity();
  }

  /**
   * Retrieves the current position of the motor.
   *
   * @return The motor position as a Rotation2d object.
   */
  public Rotation2d getPosition() {
    determineConversions(false);
    double position = positionConversion.fromMotor(mMotor.getPosition()) * positionConversionFactor;
    return Rotation2d.fromDegrees(position);
  }

  /**
   * Retrieves the current velocity of the motor.
   *
   * @return The motor velocity in meters per second.
   */
  public double getVelocity() {

    determineConversions(false);
    return velocityConversion.fromMotor(mMotor.getVelocity()) * velocityConversionFactor;
  }

  /**
   * Sets the motor speed based on the desired speed in meters per second.
   *
   * @param speedInMPS The desired speed in meters per second.
   */
  public void setSpeed(double speedInMPS) {
    determineConversions(true);
    double output = motorFeedFordward.calculate(speedInMPS);
    mMotor.setVelocity(output);
  }

  /**
   * Sets the motor speed based on the desired speed and mechanism circumference.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param mechanismCircumference The circumference of the mechanism connected to the motor.
   */
  public void setSpeed(double speedInMPS, double mechanismCircumference) {
    determineConversions(true);
    double velocity = speedInMPS / mechanismCircumference;
    double output = motorFeedFordward.calculate(velocity);
    mMotor.setVelocity(output);
  }

  /**
   * Sets the motor angle to the specified value in rotations.
   *
   * @param angleInRotations The desired angle in rotations.
   */
  public void setAngle(double angleInRotations) {
    determineConversions(false);
    mMotor.setEncoderPosition(positionConversion.toMotor(angleInRotations));
  }

  /**
   * Sets the motor control voltage.
   *
   * @param volts The voltage to apply to the motor, capped at 12V.
   */
  public void setControl(double volts) {
    if (volts > 12) {
      volts = 12;
    }
    double output = volts / 12;
    mMotor.set(output);
  }

  /** Stops the motor. */
  public void stopMotor() {
    mMotor.stopMotor();
  }

  /** Checks for any faults in the motor. */
  public void checkForFaults() {
    faults.hasFaultOccured();
  }

  /**
   * Retrieves the SparkMax motor instance.
   *
   * @return The SparkMax instance representing the motor.
   */
  public ThriftyNova getMotor() {
    return mMotor;
  }

  /**
   * Retrieves the CAN ID of the motor.
   *
   * @return The CAN ID of the motor.
   */
  public int getCanId() {
    return motorCanId;
  }
}
