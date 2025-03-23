package GalvonizedLib.Subsystems.Swerve.Subsystems.vision.util.apriltag;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.UncheckedIOException;

/** Loadable AprilTag field layouts. */
public enum AprilTagFields {
  /** 2022 Rapid React. */
  k2022RapidReact("2022-rapidreact.json"),
  /** 2023 Charged Up. */
  k2023ChargedUp("2023-chargedup.json"),
  /** 2024 Crescendo. */
  k2024Crescendo("2024-crescendo.json"),
  /** 2025 Reefscape. */
  k2025Reefscape("2025-reefscape.json");

  /** Base resource directory. */
  public static String kBaseResourceDir = "";

  /** Alias to the current game. */
  public static final AprilTagFields kDefaultField = k2025Reefscape;

  /** Resource filename. */
  public final String m_resourceFile;

  AprilTagFieldLayout m_fieldLayout;

  AprilTagFields(String resourceFile) {
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/apriltag/");
    m_resourceFile = directory.getPath() + resourceFile;
  }

  /**
   * Get a {@link AprilTagFieldLayout} from the resource JSON.
   *
   * @return AprilTagFieldLayout of the field
   * @throws UncheckedIOException If the layout does not exist
   * @deprecated Use {@link AprilTagFieldLayout#loadField(AprilTagFields)} instead.
   */
  @Deprecated(forRemoval = true, since = "2025")
  public AprilTagFieldLayout loadAprilTagLayoutField() {
    return AprilTagFieldLayout.loadField(this);
  }
}
