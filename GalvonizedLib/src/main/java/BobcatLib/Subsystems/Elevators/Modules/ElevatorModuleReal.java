package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Hardware.Motors.Utility.CTRE.MotionMagicWrapper;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import BobcatLib.Subsystems.Elevators.Modules.Motors.BaseElevatorMotor;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;

/**
 * Implementation of the ElevatorModule interface for a real elevator module. Handles motor control,
 * position tracking, and voltage control.
 */
public class ElevatorModuleReal implements ElevatorModuleIO {
  private BaseElevatorMotor motor;
  private double currentSetPoint;
  private VoltageOut voltageRequest = new VoltageOut(0);

  /** Default constructor. */
  public ElevatorModuleReal() {}

  /**
   * Constructor that initializes the elevator module with software limits and Motion Magic.
   *
   * @param limits Software limit wrapper for motor control.
   * @param mm Motion Magic configuration wrapper.
   */
  public ElevatorModuleReal(SoftwareLimitWrapper limits, MotionMagicWrapper mm) {
    motor = motor.withSoftwareLimit(limits);
    motor = motor.withMotionMagic(mm);
  }

  /**
   * Constructor that initializes the elevator module with Motion Magic.
   *
   * @param mm Motion Magic configuration wrapper.
   */
  public ElevatorModuleReal(MotionMagicWrapper mm) {
    motor = motor.withMotionMagic(mm);
  }

  /**
   * Constructor that initializes the elevator module with software limits.
   *
   * @param limits Software limit wrapper for motor control.
   */
  public ElevatorModuleReal(SoftwareLimitWrapper limits) {
    motor = motor.withSoftwareLimit(limits);
  }

  /**
   * Updates the input values for the elevator module.
   *
   * @param inputs Elevator input structure containing position and setpoint data.
   */
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorPosition = getPosition().getRotations();
    inputs.currentSetPoint = currentSetPoint;
  }

  /**
   * Moves the elevator to a specified setpoint.
   *
   * @param setPoint The target position for the elevator.
   */
  public void moveElevator(Rotation2d setPoint) {
    motor.setAngle(setPoint.getRotations());
  }

  /**
   * Runs the elevator at a specified velocity.
   *
   * @param velocity The velocity at which to run the elevator.
   */
  public void runElevator(double velocity) {
    motor.setControl(velocity);
  }

  /**
   * Gets the current position of the elevator.
   *
   * @return The elevator position as a Rotation2d object.
   */
  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(motor.getPosition());
  }

  /** Stops the elevator motor immediately. */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Sets the current setpoint for the elevator position.
   *
   * @param rotations The setpoint in rotations.
   */
  public void setCurrentSetPoint(double rotations) {
    currentSetPoint = rotations;
  }

  /**
   * Runs the elevator using a specified voltage.
   *
   * @param volts The voltage to apply to the motor.
   */
  public void runVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
