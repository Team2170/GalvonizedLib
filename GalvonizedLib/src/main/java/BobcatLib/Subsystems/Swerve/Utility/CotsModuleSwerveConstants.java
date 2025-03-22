package BobcatLib.Subsystems.Swerve.Utility;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.SensorDirectionWrapper;
import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class CotsModuleSwerveConstants {
  /** wheelDiameter */
  public final double wheelDiameter;
  /** wheelCircumference */
  public final double wheelCircumference;
  /** driveGearRatio */
  public final double angleGearRatio;
  /** wheelDiameter */
  public final double driveGearRatio;
  /** angleKP */
  public final double angleKP;
  /** angleKI */
  public final double angleKI;
  /** angleKD */
  public final double angleKD;
  /** driveMotorInvert */
  public final InvertedWrapper driveMotorInvert;
  /** angleMotorInvert */
  public final InvertedWrapper angleMotorInvert;
  /** absoluteEncoderInvert */
  public final SensorDirectionWrapper absoluteEncoderInvert;

  /**
   * @param wheelDiameter wheelDiameter
   * @param angleGearRatio angle Gear
   * @param driveGearRatio drive Gear
   * @param angleKP angle kp
   * @param angleKI angle ki
   * @param angleKD angle kd
   * @param driveMotorInvert drive invert
   * @param angleMotorInvert angle invert
   * @param cancoderInvert cancoder invert
   */
  public CotsModuleSwerveConstants(
      double wheelDiameter,
      double angleGearRatio,
      double driveGearRatio,
      double angleKP,
      double angleKI,
      double angleKD,
      InvertedWrapper driveMotorInvert,
      InvertedWrapper angleMotorInvert,
      SensorDirectionWrapper cancoderInvert) {
    this.wheelDiameter = wheelDiameter;
    this.wheelCircumference = wheelDiameter * Math.PI;
    this.angleGearRatio = angleGearRatio;
    this.driveGearRatio = driveGearRatio;
    this.angleKP = angleKP;
    this.angleKI = angleKI;
    this.angleKD = angleKD;
    this.driveMotorInvert = driveMotorInvert;
    this.angleMotorInvert = angleMotorInvert;
    this.absoluteEncoderInvert = cancoderInvert;
  }

  /** West Coast Products */
  public static final class WCP {
    /** West Coast Products - SwerveX Standard */
    public static final class SwerveXStandard {
      /**
       * West Coast Products - SwerveX Standard (Falcon 500)
       *
       * @param driveGearRatio Drove Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (396 / 35) : 1 */
        double angleGearRatio = ((396.0 / 35.0) / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX Standard (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (396 / 35) : 1 */
        double angleGearRatio = ((396.0 / 35.0) / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX Standard (Neo)
       *
       * @param driveGearRatio Drive Ratios
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (396 / 35) : 1 */
        double angleGearRatio = ((396.0 / 35.0) / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX Standard (Vortex)
       *
       * @param driveGearRatio Drive Ratios
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (396 / 35) : 1 */
        double angleGearRatio = ((396.0 / 35.0) / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Ratio */
      public static final class driveRatios {
        /** WCP SwerveX Standard X1 - 10 Tooth - (7.85 : 1) */
        public static final double X1_10 = (7.85 / 1.0);

        /** WCP SwerveX Standard X1 - 11 Tooth - (7.13 : 1) */
        public static final double X1_11 = (7.13 / 1.0);

        /** WCP SwerveX Standard X1 - 12 Tooth - (6.54 : 1) */
        public static final double X1_12 = (6.54 / 1.0);

        /** WCP SwerveX Standard X2 - 10 Tooth - (6.56 : 1) */
        public static final double X2_10 = (6.56 / 1.0);

        /** WCP SwerveX Standard X2 - 11 Tooth - (5.96 : 1) */
        public static final double X2_11 = (5.96 / 1.0);

        /** WCP SwerveX Standard X2 - 12 Tooth - (5.46 : 1) */
        public static final double X2_12 = (5.46 / 1.0);

        /** WCP SwerveX Standard X3 - 12 Tooth - (5.14 : 1) */
        public static final double X3_12 = (5.14 / 1.0);

        /** WCP SwerveX Standard X3 - 13 Tooth - (4.75 : 1) */
        public static final double X3_13 = (4.75 / 1.0);

        /** WCP SwerveX Standard X3 - 14 Tooth - (4.41 : 1) */
        public static final double X3_14 = (4.41 / 1.0);
      }
    }

    /** West Coast Products - SwerveX Flipped */
    public static final class SwerveXFlipped {
      /**
       * West Coast Products - SwerveX Flipped (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (468 / 35) : 1 */
        double angleGearRatio = ((468.0 / 35.0) / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX Flipped (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (468 / 35) : 1 */
        double angleGearRatio = ((468.0 / 35.0) / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX Flipped (Neo)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (468 / 35) : 1 */
        double angleGearRatio = ((468.0 / 35.0) / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX Flipped (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (468 / 35) : 1 */
        double angleGearRatio = ((468.0 / 35.0) / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Gear Ratio */
      public static final class driveRatios {
        /** WCP SwerveX Flipped X1 - 10 Tooth - (8.10 : 1) */
        public static final double X1_10 = (8.10 / 1.0);

        /** WCP SwerveX Flipped X1 - 11 Tooth - (7.36 : 1) */
        public static final double X1_11 = (7.36 / 1.0);

        /** WCP SwerveX Flipped X1 - 12 Tooth - (6.75 : 1) */
        public static final double X1_12 = (6.75 / 1.0);

        /** WCP SwerveX Flipped X2 - 10 Tooth - (6.72 : 1) */
        public static final double X2_10 = (6.72 / 1.0);

        /** WCP SwerveX Flipped X2 - 11 Tooth - (6.11 : 1) */
        public static final double X2_11 = (6.11 / 1.0);

        /** WCP SwerveX Flipped X2 - 12 Tooth - (5.60 : 1) */
        public static final double X2_12 = (5.60 / 1.0);

        /** WCP SwerveX Flipped X3 - 10 Tooth - (5.51 : 1) */
        public static final double X3_10 = (5.51 / 1.0);

        /** WCP SwerveX Flipped X3 - 11 Tooth - (5.01 : 1) */
        public static final double X3_11 = (5.01 / 1.0);

        /** WCP SwerveX Flipped X3 - 12 Tooth - (4.59 : 1) */
        public static final double X3_12 = (4.59 / 1.0);
      }
    }

    /** West Coast Products - SwerveX2St */
    public static final class SwerveX2St {
      /**
       * West Coast Products -SwerveX2St (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (25.9) : 1 */
        double angleGearRatio = (25.9 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2St (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (25.9) : 1 */
        double angleGearRatio = (25.9 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2St (Neo)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (25.9) : 1 */
        double angleGearRatio = (25.9 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2St (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (25.9) : 1 */
        double angleGearRatio = (25.9 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Gear Ratio */
      public static final class driveRatios {
        /** WCP SwerveX2St X1 - 15 Tooth - (6 : 1) */
        public static final double X1_15 = (6 / 1.0);

        /** WCP SwerveX2St X1 - 16 Tooth - (5.63 : 1) */
        public static final double X1_16 = (5.63 / 1.0);

        /** WCP SwerveX2St X1 - 17 Tooth - (5.29 : 1) */
        public static final double X1_17 = (5.29 / 1.0);

        /** WCP SwerveX2St X2 - 17 Tooth - (4.94 : 1) */
        public static final double X2_17 = (4.94 / 1.0);

        /** WCP SwerveX2St X2 - 18 Tooth - (4.67 : 1) */
        public static final double X2_18 = (4.67 / 1.0);

        /** WCP SwerveX2St X2 - 19 Tooth - (4.42 : 1) */
        public static final double X2_19 = (4.42 / 1.0);

        /** WCP SwerveX2St X3 - 19 Tooth - (4.11 : 1) */
        public static final double X3_19 = (4.11 / 1.0);

        /** WCP SwerveX2St X3 - 20 Tooth - (3.9 : 1) */
        public static final double X3_20 = (3.9 / 1.0);

        /** WCP SwerveX2St X3 - 21 Tooth - (3.71 : 1) */
        public static final double X3_21 = (3.71 / 1.0);
      }
    }

    /** West Coast Products - SwerveX2S */
    public static final class SwerveX2S {
      /**
       * West Coast Products -SwerveX2S (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (25.9) : 1 */
        double angleGearRatio = (25.9 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2S (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (25.9) : 1 */
        double angleGearRatio = (25.9 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2S (Neo)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (25.9) : 1 */
        double angleGearRatio = (25.9 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2S (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (25.9) : 1 */
        double angleGearRatio = (25.9 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Gear Ratio */
      public static final class driveRatios {
        /** WCP SwerveX2S X1 - 15 Tooth - (6 : 1) */
        public static final double X1_15 = (6 / 1.0);

        /** WCP SwerveX2S X1 - 16 Tooth - (5.63 : 1) */
        public static final double X1_16 = (5.63 / 1.0);

        /** WCP SwerveX2S X1 - 17 Tooth - (5.29 : 1) */
        public static final double X1_17 = (5.29 / 1.0);

        /** WCP SwerveX2S X2 - 17 Tooth - (4.94 : 1) */
        public static final double X2_17 = (4.94 / 1.0);

        /** WCP SwerveX2S X2 - 18 Tooth - (4.67 : 1) */
        public static final double X2_18 = (4.67 / 1.0);

        /** WCP SwerveX2S X2 - 19 Tooth - (4.42 : 1) */
        public static final double X2_19 = (4.42 / 1.0);

        /** WCP SwerveX2S X3 - 19 Tooth - (4.11 : 1) */
        public static final double X3_19 = (4.11 / 1.0);

        /** WCP SwerveX2S X3 - 20 Tooth - (3.9 : 1) */
        public static final double X3_20 = (3.9 / 1.0);

        /** WCP SwerveX2S X3 - 21 Tooth - (3.71 : 1) */
        public static final double X3_21 = (3.71 / 1.0);
      }
    }

    /** West Coast Products - SwerveX2t */
    public static final class SwerveX2t {
      /**
       * West Coast Products -SwerveX2t (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2St (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2t (Neo)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2t (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Gear Ratio */
      public static final class driveRatios {
        /** WCP SwerveX2t X1 - 10 Tooth - (7.67 : 1) */
        public static final double X1_10 = (7.67 / 1.0);

        /** WCP SwerveX2t X1 - 11 Tooth - (6.98 : 1) */
        public static final double X1_11 = (6.98 / 1.0);

        /** WCP SwerveX2t X1 - 12 Tooth - (6.39 : 1) */
        public static final double X1_12 = (6.39 / 1.0);

        /** WCP SwerveX2t X2 - 10 Tooth - (6.82 : 1) */
        public static final double X2_10 = (6.82 / 1.0);

        /** WCP SwerveX2t X2 - 11 Tooth - (6.20 : 1) */
        public static final double X2_11 = (6.20 / 1.0);

        /** WCP SwerveX2t X2 - 12 Tooth - (5.68 : 1) */
        public static final double X2_12 = (5.68 / 1.0);

        /** WCP SwerveX2t X3 - 10 Tooth - (6.48 : 1) */
        public static final double X3_10 = (6.48 / 1.0);

        /** WCP SwerveX2t X3 - 11 Tooth - (5.89 : 1) */
        public static final double X3_11 = (5.89 / 1.0);

        /** WCP SwerveX2t X3 - 12 Tooth - (5.40 : 1) */
        public static final double X3_12 = (3.71 / 1.0);

        /** WCP SwerveX2t X4 - 10 Tooth - (5.67 : 1) */
        public static final double X4_10 = (5.67 / 1.0);

        /** WCP SwerveX2t X4 - 11 Tooth - (5.15 : 1) */
        public static final double X4_11 = (5.15 / 1.0);

        /** WCP SwerveX2t X4 - 12 Tooth - (4.73 : 1) */
        public static final double X4_12 = (4.73 / 1.0);
      }
    }

    /** West Coast Products - SwerveX2 */
    public static final class SwerveX2 {
      /**
       * West Coast Products -SwerveX2 (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2 (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2 (Neo)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2 (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Gear Ratio */
      public static final class driveRatios {
        /** WCP SwerveX2t X1 - 10 Tooth - (7.67 : 1) */
        public static final double X1_10 = (7.67 / 1.0);

        /** WCP SwerveX2t X1 - 11 Tooth - (6.98 : 1) */
        public static final double X1_11 = (6.98 / 1.0);

        /** WCP SwerveX2t X1 - 12 Tooth - (6.39 : 1) */
        public static final double X1_12 = (6.39 / 1.0);

        /** WCP SwerveX2t X2 - 10 Tooth - (6.82 : 1) */
        public static final double X2_10 = (6.82 / 1.0);

        /** WCP SwerveX2t X2 - 11 Tooth - (6.20 : 1) */
        public static final double X2_11 = (6.20 / 1.0);

        /** WCP SwerveX2t X2 - 12 Tooth - (5.68 : 1) */
        public static final double X2_12 = (5.68 / 1.0);

        /** WCP SwerveX2t X3 - 10 Tooth - (6.48 : 1) */
        public static final double X3_10 = (6.48 / 1.0);

        /** WCP SwerveX2t X3 - 11 Tooth - (5.89 : 1) */
        public static final double X3_11 = (5.89 / 1.0);

        /** WCP SwerveX2t X3 - 12 Tooth - (5.40 : 1) */
        public static final double X3_12 = (3.71 / 1.0);

        /** WCP SwerveX2t X4 - 10 Tooth - (5.67 : 1) */
        public static final double X4_10 = (5.67 / 1.0);

        /** WCP SwerveX2t X4 - 11 Tooth - (5.15 : 1) */
        public static final double X4_11 = (5.15 / 1.0);

        /** WCP SwerveX2t X4 - 12 Tooth - (4.73 : 1) */
        public static final double X4_12 = (4.73 / 1.0);
      }
    }

    /** West Coast Products - SwerveX2i */
    public static final class SwerveX2i {
      /**
       * West Coast Products -SwerveX2i (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2i (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 1.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2 (Neo)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * West Coast Products - SwerveX2i (Vortex)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(3.5);

        /** (12.1) : 1 */
        double angleGearRatio = (12.1 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Gear Ratio */
      public static final class driveRatios {
        /** WCP SwerveX2t X1 - 10 Tooth - (7.67 : 1) */
        public static final double X1_10 = (7.67 / 1.0);

        /** WCP SwerveX2t X1 - 11 Tooth - (6.98 : 1) */
        public static final double X1_11 = (6.98 / 1.0);

        /** WCP SwerveX2t X1 - 12 Tooth - (6.39 : 1) */
        public static final double X1_12 = (6.39 / 1.0);

        /** WCP SwerveX2t X2 - 10 Tooth - (6.82 : 1) */
        public static final double X2_10 = (6.82 / 1.0);

        /** WCP SwerveX2t X2 - 11 Tooth - (6.20 : 1) */
        public static final double X2_11 = (6.20 / 1.0);

        /** WCP SwerveX2t X2 - 12 Tooth - (5.68 : 1) */
        public static final double X2_12 = (5.68 / 1.0);

        /** WCP SwerveX2t X3 - 10 Tooth - (6.48 : 1) */
        public static final double X3_10 = (6.48 / 1.0);

        /** WCP SwerveX2t X3 - 11 Tooth - (5.89 : 1) */
        public static final double X3_11 = (5.89 / 1.0);

        /** WCP SwerveX2t X3 - 12 Tooth - (5.40 : 1) */
        public static final double X3_12 = (3.71 / 1.0);

        /** WCP SwerveX2t X4 - 10 Tooth - (5.67 : 1) */
        public static final double X4_10 = (5.67 / 1.0);

        /** WCP SwerveX2t X4 - 11 Tooth - (5.15 : 1) */
        public static final double X4_11 = (5.15 / 1.0);

        /** WCP SwerveX2t X4 - 12 Tooth - (4.73 : 1) */
        public static final double X4_12 = (4.73 / 1.0);
      }
    }
  }

  /** Swerve Drive Specialities */
  public static final class SDS {
    /** Swerve Drive Specialties - MK3 Module */
    public static final class MK3 {
      /**
       * Swerve Drive Specialties - MK3 Module (Falcon 500)
       *
       * @param driveGearRatio Drive Ratios
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(false);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK3 Module (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(false);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK3 Module (NEO)
       *
       * @param driveGearRatio drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(false);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK3 Module (NEO)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(false);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Ratios */
      public static final class driveRatios {
        /** SDS MK3 - (8.16 : 1) */
        public static final double Standard = (8.16 / 1.0);
        /** SDS MK3 - (6.86 : 1) */
        public static final double Fast = (6.86 / 1.0);
      }
    }

    /** Swerve Drive Specialties - MK4 Module */
    public static final class MK4 {
      /**
       * Swerve Drive Specialties - MK4 Module (Falcon 500)
       *
       * @param driveGearRatio drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(false);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4 Module (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(false);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4 Module (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(false);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4 Module (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(false);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Ratios */
      public static final class driveRatios {
        /** SDS MK4 - (8.14 : 1) */
        public static final double L1 = (8.14 / 1.0);
        /** SDS MK4 - (6.75 : 1) */
        public static final double L2 = (6.75 / 1.0);
        /** SDS MK4 - (6.12 : 1) */
        public static final double L3 = (6.12 / 1.0);
        /** SDS MK4 - (5.14 : 1) */
        public static final double L4 = (5.14 / 1.0);
      }
    }

    /** Swerve Drive Specialties - MK4i Module */
    public static final class MK4i {

      /**
       * Swerve Drive Specialties - MK4i Module (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;
        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Kraken X60
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 0.1;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Ratios */
      public static final class driveRatios {
        /** SDS MK4i - (8.14 : 1) */
        public static final double L1 = (8.14 / 1.0);
        /** SDS MK4i - (6.75 : 1) */
        public static final double L2 = (6.75 / 1.0);
        /** SDS MK4i - (6.12 : 1) */
        public static final double L3 = (6.12 / 1.0);
        /** SDS MK4i - (5.36 : 1) 16T upgrade */
        public static final double L4 = (5.36 / 1.0);
      }
    }

    /** Swerve Drive Specialties - MK4n Module */
    public static final class MK4n {

      /**
       * Swerve Drive Specialties - MK4i Module (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (18.75 / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (18.75 / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (18.75 / 1.0);

        double angleKP = 0.01;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (18.75 / 1.0);

        double angleKP = 0.01;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Ratios */
      public static final class driveRatios {
        /** SDS MK4n - (7.13 : 1) */
        public static final double L1 = (7.13 / 1.0);
        /** SDS MK4n - (5.9 : 1) */
        public static final double L2 = (5.9 / 1.0);
        /** SDS MK4n - (5.36 : 1) */
        public static final double L3 = (5.36 / 1.0);
      }
    }

    /** Swerve Drive Specialties - MK4n Module */
    public static final class MK4c {

      /**
       * Swerve Drive Specialties - MK4i Module (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Falcon500(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants KrakenX60(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 100.0;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Falcon 500)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Neo(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.01;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /**
       * Swerve Drive Specialties - MK4i Module (Kraken X60)
       *
       * @param driveGearRatio Drive Gear Ratio
       * @return COTSSwerveConstants
       */
      public static final CotsModuleSwerveConstants Vortex(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (12.8 / 1.0);

        double angleKP = 0.01;
        double angleKI = 0.0;
        double angleKD = 0.0;

        InvertedWrapper driveMotorInvert = new InvertedWrapper(false);
        InvertedWrapper angleMotorInvert = new InvertedWrapper(true);
        SensorDirectionWrapper cancoderInvert = new SensorDirectionWrapper(false);
        return new CotsModuleSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      /** Drive Ratios */
      public static final class driveRatios {
        /** SDS MK4n - (7.13 : 1) */
        public static final double L1 = (7.13 / 1.0);
        /** SDS MK4n - (5.9 : 1) */
        public static final double L2 = (5.9 / 1.0);
        /** SDS MK4n - (5.36 : 1) */
        public static final double L3 = (5.36 / 1.0);
      }
    }
  }
}
