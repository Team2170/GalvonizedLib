package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.SensorDirectionWrapper;
import BobcatLib.Utilities.CANDeviceDetails;

/**
 * A configuration class for motor settings. This class holds the various parameters and settings
 * that define the motor configuration, including control gains, sensor direction, motor state, and
 * other optional configurations.
 */
public class MotorConfigs {

  /** The gear ratio between the motor and the connected gear. */
  public double motorToGearRatio;

  /** The neutral mode of the motor, determining the behavior when the motor is disabled. */
  public NeutralModeWrapper.disabledMode mode;

  /** The direction of the motor sensor (inverted or not). */
  public SensorDirectionWrapper sensorDirection;

  /** Indicates whether the motor is inverted or not. */
  public boolean isInverted;

  /** The proportional gain (kP) for motor control. */
  public double kP;

  /** The integral gain (kI) for motor control. */
  public double kI;

  /** The derivative gain (kD) for motor control. */
  public double kD;

  /** Optional configuration for the CTRE motor controller. */
  public OptionalConfigCtre optionalCtre = new OptionalConfigCtre();

  /** Optional configuration for the REV motor controller. */
  public OptionalConfigRev optionalRev = new OptionalConfigRev();

  /** The CAN device details, including device number and bus. */
  public CANDeviceDetails canDevice;
}
