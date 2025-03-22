package BobcatLib.Subsystems.Climbers.Modules;

import BobcatLib.Hardware.Motors.BaseMotor;
import BobcatLib.Hardware.Motors.MotorIO;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Real implementation of the {@link ClimberIO} interface for controlling a physical climber
 * mechanism using motor control.
 */
public class ClimberReal implements ClimberIO {

  /** The motor controlling the climber mechanism. */
  private BaseMotor climberMotor;

  private SoftwareLimitWrapper limits;

  /**
   * Constructs a {@code ClimberReal} instance with the specified motor configuration.
   *
   * @param motor The {@link MotorIO} object for configuring the climber motor.
   */
  public ClimberReal(MotorIO motor, SoftwareLimitWrapper limits) {
    this.limits = limits;
    climberMotor = new BaseMotor(motor, "Climber/Motor", limits);
  }

  /**
   * Configures the climber motor and sets up limits and control parameters.
   *
   * <p>This method should override the motor configuration in the {@link BaseMotor} class. Tasks
   * include setting reverse and forward soft limits for upper and lower bounds, enabling
   * proportional gain (kP), and applying current limits.
   */
  public void configClimber() {}

  /**
   * Updates the input values of the climber, such as the current motor position.
   *
   * @param inputs The {@link ClimberIOInputs} object containing updated input data.
   */
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberMotorPosition = getPosition().getRotations();
  }

  /**
   * Sets the motor output as a percentage of its maximum power.
   *
   * <p>This value must be between -1.0 (full reverse) and 1.0 (full forward). No validation is
   * performed on the input, so users must ensure the value is within the valid range.
   *
   * @param percent The desired motor output percentage (-1.0 to 1.0).
   */
  public void setPercentOut(double percent) {
    climberMotor.setControl(percent);
  }

  /**
   * Commands the climber motor to hold a specific position.
   *
   * @param rot The target position in rotations.
   */
  public void holdPos(double rot) {
    climberMotor.setAngle(rot);
  }

  /** Stops the climber motor immediately. */
  public void stop() {
    climberMotor.stopMotor();
  }

  /**
   * Retrieves the current position of the climber mechanism.
   *
   * @return The current position as a {@link Rotation2d}.
   */
  public Rotation2d getPosition() {
    return climberMotor.getPosition();
  }
}
