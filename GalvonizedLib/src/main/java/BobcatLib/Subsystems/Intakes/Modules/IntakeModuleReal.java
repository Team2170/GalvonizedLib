package BobcatLib.Subsystems.Intakes.Modules;

import BobcatLib.Hardware.Encoders.BaseEncoder;
import BobcatLib.Hardware.Encoders.EncoderIO;
import BobcatLib.Hardware.Motors.BaseMotor;
import BobcatLib.Hardware.Motors.MotorIO;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import BobcatLib.Subsystems.Intakes.Modules.IntakeModuleIO.IntakeIOInputs;
import BobcatLib.Subsystems.Intakes.Parser.IntakeJson;
import BobcatLib.Subsystems.Intakes.Utils.IntakeConstants;
import BobcatLib.Subsystems.Intakes.Utils.IntakeMotorConstants;
import BobcatLib.Subsystems.Intakes.Utils.IntakeState;
import BobcatLib.Subsystems.Intakes.Utils.IntakeType;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface for the Intake Module as a real implementation. Defines methods to interact with and
 * control the intake mechanism, including setting states, managing positions, and obtaining sensor
 * data.
 */
public class IntakeModuleReal {
  private BaseMotor rollerMotor;
  private BaseMotor pivotMotor;
  private BaseEncoder intakeAbsEncoder;
  private IntakeType intakeType;
  private IntakeConstants configuration;
  private SoftwareLimitWrapper limits;

  /**
   * This implementation should be used when you have a static intake that never moves with a
   * pivoting motor;
   *
   * @param rollerMotor
   */
  public IntakeModuleReal(MotorIO rollerMotor, IntakeJson intakeJson, SoftwareLimitWrapper limits) {
    loadConfigurationFromFile(intakeJson);
    this.limits = limits;
    this.rollerMotor = new BaseMotor(rollerMotor, "Intake/Motor", limits);
    intakeType = IntakeType.STATIONARY;
  }

  /**
   * This implementation should be used with a pivoting intake where you do not have an absolute
   * encoder attached to the pivot angle.
   *
   * @param rollerMotor
   * @param pivotMotor
   */
  public IntakeModuleReal(MotorIO rollerMotor, MotorIO pivotMotor, IntakeJson intakeJson) {
    loadConfigurationFromFile(intakeJson);
    this.rollerMotor = new BaseMotor(rollerMotor, "Intake/Motor", limits);
    this.pivotMotor = new BaseMotor(pivotMotor, "Intake/Motor", limits);
    intakeType = IntakeType.PIVOTING;
  }

  /**
   * This implementation should be used with a pivoting intake where you have an absolute encoder
   * attached to the pivot angle.
   *
   * @param rollerMotor
   * @param pivotMotor
   * @param absEncoder
   */
  public IntakeModuleReal(
      MotorIO rollerMotor, MotorIO pivotMotor, EncoderIO absEncoder, IntakeJson intakeJson) {
    loadConfigurationFromFile(intakeJson);
    this.rollerMotor = new BaseMotor(rollerMotor, "Intake/Motor", limits);
    this.pivotMotor = new BaseMotor(pivotMotor, "Intake/Motor", limits);
    this.intakeAbsEncoder = new BaseEncoder(absEncoder);
    intakeType = IntakeType.PIVOTING;
  }

  public void loadConfigurationFromFile(IntakeJson intakeJson) {
    IntakeMotorConstants rollerMotorConstants =
        new IntakeMotorConstants(
            intakeJson.rollerMotorId,
            intakeJson.rollerPid_kP,
            intakeJson.rollerPid_kI,
            intakeJson.rollerPid_kD);
    IntakeMotorConstants pivotMotorConstants = new IntakeMotorConstants();
    if (intakeJson.usePivotMotor) {
      pivotMotorConstants =
          new IntakeMotorConstants(
              intakeJson.pivotMotorId,
              intakeJson.pivotPid_kP,
              intakeJson.pivotPid_kI,
              intakeJson.pivotPid_kD,
              intakeJson.useAbsEncoder,
              intakeJson.angleOffset,
              intakeJson.absPivotEncoderId,
              intakeJson.mechanismCircumference,
              intakeJson.upperPivotRange,
              intakeJson.lowerPivotRange,
              intakeJson.pivotMotorInverted,
              intakeJson.pivotBrakeMode);
    }
    configuration = new IntakeConstants(intakeJson.name, rollerMotorConstants, pivotMotorConstants);
  }

  /**
   * Updates the input values of the intake module.
   *
   * @param inputs The inputs to update.
   */
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotAngle = getPosition();
    inputs.rollerVelocity = getVelocity();
  }

  /**
   * Sets the overall state of the intake mechanism.
   *
   * @param state The desired state of the intake mechanism.
   */
  public void setIntakeState(IntakeState state) {
    if (intakeType == IntakeType.STATIONARY) {
      setSingleModeState(state);
      return;
    }
    setPivotingModeState(state);
  }

  /**
   * Sets the intake mechanism to operate in single mode.
   *
   * @param state The desired state for single mode.
   */
  public void setSingleModeState(IntakeState state) {
    double velocity = state.getRollerVelocity();
    setVelocity(
        velocity, configuration.getPivotMotorConstants().getMechanismCircumference(), false);
  }

  /**
   * Sets the intake mechanism to operate in pivoting mode.
   *
   * @param state The desired state for pivoting mode.
   */
  public void setPivotingModeState(IntakeState state) {
    double angle = state.getPivotAngle().getRotations();
    setAngle(angle);
  }

  /**
   * Sets the pivot angle of the intake mechanism.
   *
   * @param rotations The desired angle in rotations.
   */
  public void setAngle(double rotations) {
    pivotMotor.setAngle(rotations);
  }

  /**
   * Sets the velocity of the intake roller.
   *
   * @param speedInMPS The desired velocity in meters per second.
   * @param mechanismCircumference The circumference of the roller mechanism.
   * @param isOpen Whether the roller should be operating in open-loop mode.
   */
  public void setVelocity(double speedInMPS, double mechanismCircumference, boolean isOpen) {
    rollerMotor.setSpeed(speedInMPS, mechanismCircumference, false);
  }

  /**
   * Holds the pivot mechanism at a specific position.
   *
   * @param stopRollers The desired position in rotations.
   */
  public void holdPos(boolean stopRollers) {
    double velocity = 0.00;
    if (!stopRollers) {
      velocity = getVelocity();
    }
    setVelocity(
        velocity, configuration.getPivotMotorConstants().getMechanismCircumference(), false);
    setAngle(getPosition().getRotations());
  }

  /** Stops all movement of the intake mechanism. */
  public void stop() {
    rollerMotor.stopMotor();
    pivotMotor.stopMotor();
  }

  /**
   * Gets the current state of the intake mechanism.
   *
   * @return The current state.
   */
  public IntakeState getState() {
    return new IntakeState(getVelocity(), getPosition());
  }

  /**
   * Gets the current pivot position of the intake mechanism.
   *
   * @return The current pivot angle as a Rotation2d object. Returns an empty Rotation2d if the
   *     intake does not use a pivot motor or the pivot motor is not equipped with an encoder.
   */
  public Rotation2d getPosition() {
    // Check if the configuration specifies the use of a pivot motor
    if (!configuration.isUsePivotMotor()) {
      // Return a default Rotation2d if no pivot motor is in use
      return new Rotation2d();
    }

    // Check if an absolute encoder is used for the pivot motor
    if (configuration.getPivotMotorConstants().isUseAbsEncoder()) {
      // Use the absolute encoder to determine the position
      return Rotation2d.fromRotations(intakeAbsEncoder.getAbsolutePosition());
    }

    // Fallback: Use the pivot motor's internal position
    return pivotMotor.getPosition();
  }

  /**
   * Gets the current velocity of the intake roller.
   *
   * @return The current velocity in meters per second.
   */
  public double getVelocity() {
    return rollerMotor.getVelocity();
  }
}
