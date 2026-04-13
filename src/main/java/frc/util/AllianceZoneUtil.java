package frc.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class AllianceZoneUtil {
    // Measured
    public static final Distance centerField_x_pos = Inches.of(325.06);
    public static final Distance centerField_y_pos = Inches.of(158.32);
    private static final Distance hub_x_centerPos = Inches.of(181.56);
    private static final Distance hub_x_len = Inches.of(47);
    private static final Distance hub_z_len = Inches.of(72);

    // Constructed
    public static final Distance field_x_len = centerField_x_pos.times(2);
    public static final Distance field_y_len = centerField_y_pos.times(2);
    private static final Distance allianceZone_x_len = hub_x_centerPos.minus(hub_x_len.div(2));
    private static final Distance neutralZone_x_len = (centerField_x_pos.minus(hub_x_centerPos.plus(hub_x_len.div(2)))).times(2);
    public static final Pose3d blueHubCenter =
        new Pose3d(hub_x_centerPos, centerField_y_pos, hub_z_len, Rotation3d.kZero);
    public static final Pose3d redHubCenter =
        new Pose3d(
            field_x_len.minus(hub_x_centerPos), centerField_y_pos, hub_z_len, Rotation3d.kZero);
    public static final Pose3d redRightTarget =
        new Pose3d(
            new Translation3d(Inches.of(651.2), Inches.of(49.86), Inches.zero()), Rotation3d.kZero);

    public static final Pose3d blueLeftTarget =
        new Pose3d(
            new Translation3d(Inches.zero(), Inches.of(200.86), Inches.zero()), Rotation3d.kZero);

    public static final Pose3d redLeftTarget =
        new Pose3d(
            new Translation3d(Inches.of(651.2), Inches.of(200.86), Inches.zero()), Rotation3d.kZero);

    public static final Pose3d blueRightTarget =
        new Pose3d(new Translation3d(Inches.zero(), Inches.of(49.86), Inches.zero()), Rotation3d.kZero);
    

    enum Region {
        BlueZone(
                new Rectangle2d(
                        new Translation2d(0, 0), new Translation2d(allianceZone_x_len, field_y_len))),
        RightNeutralZone(
                new Rectangle2d(
                        new Translation2d(centerField_x_pos.minus(neutralZone_x_len.div(2.0)), Inches.zero()),
                        new Translation2d(
                                centerField_x_pos.plus(neutralZone_x_len.div(2.0)), centerField_y_pos))),
        LeftNeutralZone(
                new Rectangle2d(
                        new Translation2d(
                                centerField_x_pos.minus(neutralZone_x_len.div(2.0)), centerField_y_pos),
                        new Translation2d(centerField_x_pos.plus(neutralZone_x_len.div(2.0)), field_y_len))),
        RedZone(
                new Rectangle2d(
                        new Translation2d(field_x_len.minus(allianceZone_x_len), Inches.zero()),
                        new Translation2d(field_x_len, field_y_len)));

        private final Rectangle2d rect;

        private Region(Rectangle2d rect) {
            this.rect = rect;
        }

        public boolean contains(Pose2d pose) {
            return rect.contains(pose.getTranslation());
        }
    }

    public static Region getZone(Pose2d pose) {
        if (Region.BlueZone.contains(pose)) {
            return Region.BlueZone;
        }
        if (Region.RedZone.contains(pose)) {
            return Region.RedZone;
        }
        if (Region.LeftNeutralZone.contains(pose)) {
            return Region.LeftNeutralZone;
        }
        return Region.RightNeutralZone;
    }
}