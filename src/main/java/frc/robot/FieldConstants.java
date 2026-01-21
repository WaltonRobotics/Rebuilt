package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class FieldConstants {


    // AprilTag related constants
    public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    // Field dimensions
    public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();


    public static class Hub {
         // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height =
            Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        public static final double hubCenter =
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;


         // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint =
            new Translation3d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation3d innerCenterPoint =
            new Translation3d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                innerHeight);

        public static final Translation2d nearLeftCorner =
            new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
            new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner =
            new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner =
            new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);
    }



    public enum AprilTagLayoutType {
        OFFICIAL("2026-official");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
        this.name = name;
        }

        public AprilTagFieldLayout getLayout() {
        if (layout == null) {
            synchronized (this) {
            if (layout == null) {
                try {
                Path p = 
                Path.of(
                        "src",
                        "main",
                        "deploy",
                        "apriltags",
                        "welded",
                        name + ".json"
                        );
                layout = new AprilTagFieldLayout(p);
                layoutString = new ObjectMapper().writeValueAsString(layout);
                } catch (IOException e) {
                throw new RuntimeException(e);
                }
            }
            }
        }
        return layout;
        }

        public String getLayoutString() {
        if (layoutString == null) {
            getLayout();
        }
        return layoutString;
        }
    }
}
