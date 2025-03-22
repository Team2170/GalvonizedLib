package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor;

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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class NeoDriveMotor implements DriveWrapper {
  private SparkMax motor;
  private RelativeEncoder encoder;
  /* Feed Forward Setup */
  private final SimpleMotorFeedforward driveFeedForward;
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

  public NeoDriveMotor(
      CANDeviceDetails details, int id, ModuleConstants chosenModule, ModuleLimitsJson limits) {
    this.details = details;
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    this.limits = limits;
    double driveKS = 0.00;
    double driveKV = 0.00;
    double driveKA = 0.0;
    driveFeedForward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);
    /* Drive Motor Config */
    motor = new SparkMax(id, MotorType.kBrushless);
    encoder = motor.getEncoder();
    configDriveMotor();
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(0.0);
  }

  /** Updates the Motor Outputs */
  public void updateOutputs() {
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Drive/MotorVoltage",
        getMotorVoltage());
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Drive/RelativePosition",
        Rotation2d.fromRotations(getPosition()));
    Logger.recordOutput(
        details.getSubsysemName()
            + "/"
            + details.getBus()
            + "/"
            + details.getDeviceNumber()
            + "/Drive/Velocity",
        getVelocity());
  }

  public void configDriveMotor() {
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();
    /* Motor Inverts and Neutral Mode */
    motorConfig.encoder.inverted(chosenModule.driveMotorInvert.asREV());

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig
        .encoder
        .positionConversionFactor(chosenModule.driveConversionPositionFactor)
        .velocityConversionFactor(chosenModule.driveConversionVelocityFactor);

    IdleMode motorMode = chosenModule.driveNeutralMode.asIdleMode();
    motorConfig.idleMode(motorMode);

    /* PID Config */
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(chosenModule.drivePID.kP, ClosedLoopSlot.kSlot1)
        .i(chosenModule.drivePID.kI, ClosedLoopSlot.kSlot1)
        .d(chosenModule.drivePID.kD, ClosedLoopSlot.kSlot1)
        .velocityFF(0);

    /* Open and Closed Loop Ramping */
    motorConfig.closedLoopRampRate(chosenModule.json.closedLoopRamp);
    motorConfig.openLoopRampRate(chosenModule.json.openLoopRamp);
  }

  public void withSupplyCurrent() {
    /* Current Limiting */
    motorConfig.smartCurrentLimit(chosenModule.json.driveSupplyCurrentLimit);
  }

  public void withStatorCurrent() {}

  /**
   * gets the velocity of the drive motor in rotations per second either using percentage output (
   * dutycycleout ) or velocity control triggered by the isOpenLoop parameter
   *
   * @param desiredState
   * @param isOpenLoop
   */
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double maxSpeed = limits.maxSpeed;
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
      motor.set(percentOutput);

    } else {
      double velocity = desiredState.speedMetersPerSecond;
      double output = driveFeedForward.calculate(velocity);
      closedLoopController.setReference(output, ControlType.kVelocity, 0);
    }
    // check for faults here !;
  }

  /**
   * gets the position of the drive motor in rotations
   *
   * @return position
   */
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * gets the velocity of the drive motor in rotations per second.
   *
   * @return velocity
   */
  public double getVelocity() {
    return encoder.getVelocity() * 60;
  }

  /** Stops the motors properly. */
  public void stopMotor() {
    motor.stopMotor();
  }
}
