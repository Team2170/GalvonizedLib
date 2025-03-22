package BobcatLib.Hardware.Encoders;

import BobcatLib.Hardware.Motors.SensorHelpers.SensorDirectionWrapper;

/**
 * Holds constants related to the configuration of an encoder. This class primarily manages the
 * inversion settings for an absolute encoder.
 */
public class EncoderConstants {
  /**
   * Specifies whether the absolute encoder's direction is inverted. This wrapper facilitates
   * configuration and handling of sensor direction.
   */
  public SensorDirectionWrapper absoluteEncoderInvert;
}
