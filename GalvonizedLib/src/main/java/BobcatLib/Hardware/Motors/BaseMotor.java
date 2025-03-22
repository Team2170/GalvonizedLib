package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import BobcatLib.Utilities.CANDeviceDetails;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

/**
 * The BaseMotor class serves as a wrapper for motor functionality, providing methods for
 * controlling and monitoring motor operations. It interacts with a MotorIO implementation for
 * hardware abstraction.
 */
public class BaseMotor {

  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private SoftwareLimitWrapper limits;
  private String name;

  /**
   * Constructs a new BaseMotor instance.
   *
   * @param io The MotorIO implementation to be used for motor control and feedback.
   * @param limits A SoftwareLimitWrapper instance to handle motor software limits.
   */
  public BaseMotor(MotorIO io, String name, SoftwareLimitWrapper limits) {
    this.limits = limits;
    this.io = io;
    this.name = name;
  }

  /**
   * Constructs a new BaseMotor instance with a specified CAN device details, motor configuration,
   * motor type, and software limits.
   *
   * @param details The CAN device details for the motor.
   * @param mc The motor configurations.
   * @param motor_type The type of motor (e.g., "kraken" or "falcon").
   * @param limits A SoftwareLimitWrapper instance to handle motor software limits.
   */
  public BaseMotor(
      CANDeviceDetails details, MotorConfigs mc, String motor_type, SoftwareLimitWrapper limits) {
    this.limits = limits;
    MotorIO motor;
    switch (motor_type) {
      case "kraken":
        motor = new KrakenMotor(details, details.getBus(), mc);
        break;
      case "falcon":
        motor = new FalconMotor(details, details.getBus(), mc);
        break;
      default:
        motor = new KrakenMotor(details, details.getBus(), mc);
    }
    this.io = motor;
  }

  /**
   * Periodically updates the motor inputs and processes them. This method should be called
   * frequently to ensure the motor's state is up-to-date.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  /**
   * Retrieves the current position of the motor as a Rotation2d object.
   *
   * @return The motor's position as a Rotation2d instance.
   */
  public Rotation2d getPosition() {
    return io.getPosition();
  }

  /**
   * Retrieves the current velocity of the motor.
   *
   * @return The motor's velocity in meters per second.
   */
  public double getVelocity() {
    return io.getVelocity();
  }

  /** Stops the motor and performs a fault check to ensure no issues are present. */
  public void stopMotor() {
    io.stopMotor();
    checkForFaults();
  }

  /**
   * Sets the motor's speed.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param mechanismCircumference The circumference of the mechanism connected to the motor.
   * @param isOpenLoop Whether the speed control should be open-loop.
   */
  public void setSpeed(double speedInMPS, double mechanismCircumference, boolean isOpenLoop) {
    io.setSpeed(speedInMPS, mechanismCircumference, isOpenLoop);
    checkForFaults();
  }

  /**
   * Sets the motor's angle in rotations.
   *
   * @param angleInRotations The desired angle in rotations.
   */
  public void setAngle(double angleInRotations) {
    io.setAngle(angleInRotations);
    checkForFaults();
  }

  /**
   * Sets the motor control voltage for SysID mode.
   *
   * @param volts The control voltage to apply to the motor.
   */
  public void setControl(double volts) {
    io.setControl(volts);
    checkForFaults();
  }

  /** Performs a fault check on the motor to detect and handle any issues. */
  public void checkForFaults() {
    io.checkForFaults();
  }

  /**
   * Retrieves the MotorIO implementation associated with this BaseMotor.
   *
   * @return The MotorIO instance being used for motor control and feedback.
   */
  public MotorIO getMotor() {
    return io;
  }

  /**
   * Retrieves the CAN ID of the motor.
   *
   * @return The motor's CAN ID.
   */
  public int getCanId() {
    return io.getCanId();
  }

  /**
   * Configures the motor with both lower and upper software limits.
   *
   * @param lower The lower limit as a Rotation2d instance.
   * @param upper The upper limit as a Rotation2d instance.
   * @return The updated BaseMotor instance.
   */
  public BaseMotor withLimits(Rotation2d lower, Rotation2d upper) {
    limits =
        new SoftwareLimitWrapper(
            lower.getRotations(),
            upper.getRotations(),
            SoftwareLimitWrapper.SoftwareLimitType.BOTH);
    return this;
  }

  /**
   * Configures the motor with an upper software limit.
   *
   * @param rotations The upper limit as a Rotation2d instance.
   * @return The updated BaseMotor instance.
   */
  public BaseMotor withUpperLimit(Rotation2d rotations) {
    limits =
        new SoftwareLimitWrapper(
            rotations.getRotations(), SoftwareLimitWrapper.SoftwareLimitType.UPPER);
    return this;
  }

  /**
   * Configures the motor with a lower software limit.
   *
   * @param rotations The lower limit as a Rotation2d instance.
   * @return The updated BaseMotor instance.
   */
  public BaseMotor withLowerLimit(Rotation2d rotations) {
    limits =
        new SoftwareLimitWrapper(
            rotations.getRotations(), SoftwareLimitWrapper.SoftwareLimitType.LOWER);
    return this;
  }

  /**
   * Retrieves the upper software limit of the motor.
   *
   * @return The upper software limit as a Rotation2d instance.
   */
  public Rotation2d getUpperLimit() {
    return limits.getUpperLimit();
  }

  /**
   * Retrieves the lower software limit of the motor.
   *
   * @return The lower software limit as a Rotation2d instance.
   */
  public Rotation2d getLowerLimit() {
    return limits.getLowerLimit();
  }
}
