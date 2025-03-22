package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor;

import BobcatLib.Logging.Alert;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class NeoSteerMotor implements SteerWrapper {
  private SparkMax mAngleMotor;
  private RelativeEncoder encoder;
  /** An {@link Alert} for if the CAN ID is greater than 40. */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  public ModuleConstants chosenModule;
  public ModuleLimitsJson limits;
  private SparkMaxConfig motorConfig;

  private SparkClosedLoopController closedLoopController;
  public CANDeviceDetails details;

  public NeoSteerMotor(
      CANDeviceDetails details, int id, ModuleConstants chosenModule, ModuleLimitsJson limits) {
    this.details = details;
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    this.limits = limits;
    mAngleMotor = new SparkMax(id, MotorType.kBrushless);
    encoder = mAngleMotor.getEncoder();
    configAngleMotor();
    mAngleMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Updates the Motor Outputs */
  public void updateOutputs() {
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Angle/MotorVoltage",
        mAngleMotor.getBusVoltage());
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Angle/RelativePosition",
        Rotation2d.fromRotations(getPosition()));
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Angle/Velocity",
        encoder.getVelocity());
  }

  public void configAngleMotor() {
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /* Motor Inverts and Neutral Mode */
    motorConfig.encoder.inverted(chosenModule.angleMotorInvert.asREV());

    IdleMode motorMode = chosenModule.angleNeutralMode.asIdleMode();
    motorConfig.idleMode(motorMode);

    /* Gear Ratio Config */

    /* PID Config */
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(chosenModule.anglePID.kP, ClosedLoopSlot.kSlot1)
        .i(chosenModule.anglePID.kI, ClosedLoopSlot.kSlot1)
        .d(chosenModule.anglePID.kD, ClosedLoopSlot.kSlot1)
        .velocityFF(0);

    /* Open and Closed Loop Ramping */
    motorConfig.closedLoopRampRate(chosenModule.json.closedLoopRamp);
    motorConfig.openLoopRampRate(chosenModule.json.openLoopRamp);
  }

  public void withSupplyCurrent() {
    /* Current Limiting */
    motorConfig.smartCurrentLimit(chosenModule.json.angleSupplyCurrentLimit);
  }

  public void withStatorCurrent() {
    /* Current Limiting */
  }

  /*
   * sets the angle of the motor by setting the internal pid.
   *
   * @param rotations
   */
  public void setAngle(double rotations) {
    closedLoopController.setReference(rotations, ControlType.kPosition, 0);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * sets the position of the steer motor given the desired rotations.
   *
   * @param absolutePosition
   */
  public void setPosition(double absolutePosition) {
    encoder.setPosition(absolutePosition);
  }

  /** Stops the motors properly. */
  public void stopMotor() {
    mAngleMotor.stopMotor();
  }
}
