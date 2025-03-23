// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package GalvonizedLib.Subsystems.Swerve.Subsystems.vision;

import GalvonizedLib.Subsystems.Swerve.Subsystems.vision.util.DSUtil;
import GalvonizedLib.Subsystems.Swerve.Subsystems.vision.util.RotationUtil;
import GalvonizedLib.Subsystems.Swerve.Subsystems.vision.util.VisionObservation.LLTYPE;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {
  /** Creates a new VisionIOLimelight. */
  LEDMode currentLedMode = LEDMode.FORCEOFF;

  CamMode currentCamMode = CamMode.VISION;
  public final limelightConstants constants;
  private final String name;
  private final LLTYPE limelightType;

  public VisionIOLimelight(limelightConstants limelightConstants) {
    constants = limelightConstants;
    name = constants.name;
    limelightType = constants.limelightType;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.ledMode = currentLedMode;
    // inputs.camMode = currentCamMode;
    inputs.pipelineID = LimelightHelpers.getCurrentPipelineIndex(name);
    inputs.pipelineLatency = LimelightHelpers.getLatency_Pipeline(name);
    inputs.ta = LimelightHelpers.getTA(name);
    inputs.tv = LimelightHelpers.getTV(name);
    inputs.tx = LimelightHelpers.getTX(name); // TODO add limelight disconnect alert
    inputs.ty = LimelightHelpers.getTY(name);
    inputs.fiducialID = LimelightHelpers.getFiducialID(name);
    String llClass = LimelightHelpers.getNeuralClassID(name);
    inputs.tClass = llClass.isEmpty() ? 0 : Double.parseDouble(llClass);
    inputs.name = name;
    inputs.botPoseMG2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose;
    inputs.tagCount = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).tagCount;
    inputs.avgTagDist = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).avgTagDist;
    inputs.botPose3d = LimelightHelpers.getBotPose3d_wpiBlue(name);
    inputs.timestamp = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).timestampSeconds;
    inputs.limelightType = limelightType;
  }

  @Override
  public void setLEDS(LEDMode mode) {
    switch (mode) {
      case FORCEBLINK:
        LimelightHelpers.setLEDMode_ForceBlink(name);
        currentLedMode = LEDMode.FORCEBLINK;
        break;
      case FORCEOFF:
        LimelightHelpers.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
      case FORCEON:
        LimelightHelpers.setLEDMode_ForceOn(name);
        currentLedMode = LEDMode.FORCEON;
      case PIPELINECONTROL:
        LimelightHelpers.setLEDMode_PipelineControl(name);
        currentLedMode = LEDMode.PIPELINECONTROL;
      default:
        LimelightHelpers.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
        break;
    }
  }

  @Override
  public void setPipeline(String limelight, int index) {
    LimelightHelpers.setPipelineIndex(limelight, index);
  }

  public void setRobotOrientationMG2(Rotation3d gyro, Rotation3d rate) {
    gyro = DSUtil.isBlue() ? gyro : gyro.rotateBy(new Rotation3d(new Rotation2d(Math.PI)));
    Rotation3d gyroval = RotationUtil.wrapRot3d(gyro);
    Rotation3d rateval = RotationUtil.wrapRot3d(rate);

    LimelightHelpers.SetRobotOrientation(
        name,
        gyroval.getZ(),
        rateval.getZ(),
        gyroval.getY(),
        rateval.getY(),
        gyroval.getX(),
        rateval.getX());
  }

  @Override
  public void setPermittedTags(int[] tags) {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, tags);
  }

  @Override
  public void setPriorityID(int tagID) {
    NetworkTableInstance.getDefault().getTable(name).getEntry("priorityid").setDouble(tagID);
  }
}
