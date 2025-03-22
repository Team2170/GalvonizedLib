package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Logging.FaultsAndErrors.SparkMaxFaults;
import BobcatLib.Utilities.CANDeviceDetails;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The NeoMotor class implements the MotorIO interface and provides an abstraction for controlling a
 * Neo v1.1 motor. It handles motor initialization, configuration, and control operations such as
 * setting speed, angle, and performing fault checks.
 */
public class Neo550Motor implements MotorIO {
  private int motorCanId = 0;
  private final SimpleMotorFeedforward motorFeedFordward;
  private SparkMax mMotor;
  private RelativeEncoder encoder;
  private String busName = "";
  private SparkMaxFaults faults;
  private SparkMaxConfig motorConfig;

  private SparkClosedLoopController closedLoopController;
  private CANDeviceDetails details;

  /**
   * Constructs a NeoMotor instance with the specified CAN ID, bus name, and configuration.
   *
   * @param details The CAN details of the motor.
   * @param busname The bus name where the motor is connected.
   * @param config The motor configuration settings.
   */
  public Neo550Motor(CANDeviceDetails details, String busname, MotorConfigs config) {
    this.details = details;
    int id = details.getDeviceNumber();
    this.busName = busname;
    motorCanId = id;
    double motorKS = 0.00;
    double motorKV = 0.00;
    double motorKA = 0.00;
    motorFeedFordward = new SimpleMotorFeedforward(motorKS, motorKV, motorKA);
    /* Drive Motor Config */
    mMotor = new SparkMax(id, MotorType.kBrushless);
    encoder = mMotor.getEncoder();
    configMotor(config);
    mMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(0.0);
    faults = new SparkMaxFaults(motorCanId);
  }

  /**
   * Configures the motor settings based on the provided MotorConfigs.
   *
   * @param cfg The MotorConfigs object containing configuration parameters.
   */
  public void configMotor(MotorConfigs cfg) {

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();
    /* Motor Inverts and Neutral Mode */
    motorConfig.encoder.inverted(new InvertedWrapper(cfg.isInverted).asREV());

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig
        .encoder
        .positionConversionFactor(cfg.optionalRev.driveConversionVelocityFactor)
        .velocityConversionFactor(cfg.optionalRev.driveConversionPositionFactor);

    IdleMode motorMode = new NeutralModeWrapper(cfg.mode).asIdleMode();
    motorConfig.idleMode(motorMode);

    /* Current Limiting */
    motorConfig.smartCurrentLimit(cfg.optionalRev.SupplyCurrentLimit);

    /* PID Config */
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(cfg.kP, ClosedLoopSlot.kSlot1)
        .i(cfg.kI, ClosedLoopSlot.kSlot1)
        .d(cfg.kD, ClosedLoopSlot.kSlot1)
        .velocityFF(0);

    /* Open and Closed Loop Ramping */
    motorConfig.closedLoopRampRate(cfg.optionalRev.closedLoopRamp);
    motorConfig.openLoopRampRate(cfg.optionalRev.openLoopRamp);
  }

  /**
   * Configures the motor with software limit switch settings.
   *
   * @param cfgLimits The {@link LimitSwitchConfig} instance containing the desired limit switch
   *     configurations.
   * @return The updated {@code Neo550Motor} instance for method chaining.
   */
  public Neo550Motor withLimits(LimitSwitchConfig cfgLimits) {
    motorConfig.apply(cfgLimits);
    return this;
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
    double pos = encoder.getPosition();
    return Rotation2d.fromRotations(pos);
  }

  /**
   * Retrieves the current velocity of the motor.
   *
   * @return The motor velocity in meters per second.
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Sets the motor speed based on the desired speed in meters per second.
   *
   * @param speedInMPS The desired speed in meters per second.
   */
  public void setSpeed(double speedInMPS) {
    double output = motorFeedFordward.calculate(speedInMPS);
    closedLoopController.setReference(output, ControlType.kVelocity, 0);
  }

  /**
   * Sets the motor speed based on the desired speed and mechanism circumference.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param mechanismCircumference The circumference of the mechanism connected to the motor.
   */
  public void setSpeed(double speedInMPS, double mechanismCircumference) {
    double velocity = speedInMPS / mechanismCircumference;
    double output = motorFeedFordward.calculate(velocity);
    closedLoopController.setReference(output, ControlType.kVelocity, 0);
  }

  /**
   * Sets the motor angle to the specified value in rotations.
   *
   * @param angleInRotations The desired angle in rotations.
   */
  public void setAngle(double angleInRotations) {
    closedLoopController.setReference(angleInRotations, ControlType.kPosition, 0);
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
  public SparkMax getMotor() {
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
