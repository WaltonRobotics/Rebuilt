package frc.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionUtil {
    public static Transform3d transformToRobo(double x, double y, double z, double rotX, double rotY, double rotZ) {
        x = Units.inchesToMeters(x);
        y = Units.inchesToMeters(y);
        z = Units.inchesToMeters(z);
        rotX = Units.degreesToRadians(rotX);
        rotY = Units.degreesToRadians(rotY);
        rotZ = Units.degreesToRadians(rotZ);
        return new Transform3d(x, y, z, new Rotation3d(rotX, rotY, rotZ));
    }
}
