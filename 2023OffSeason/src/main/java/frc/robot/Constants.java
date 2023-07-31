package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class Swerve {
        public static final int pigeonID = 14;

        public static final double maxSpeed = 3.3;

        public static final double trackWidth = Units.inchesToMeters(21.25);
        public static final double wheelBase = Units.inchesToMeters(21.25);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final double accelerationTime = 0.6;
        public static final double maxAngularVelocity = (3 * Math.PI) / 2;

        public static final double stickDeadband = 0.1;

        public static final class Mod0 {
            public static final double angleOffSet = 275.80;

        }

        public static final class Mod1 {
            public static final double angleOffSet = 91.84;

        }

        public static final class Mod2 {
            public static final double angleOffSet = 329.67;

        }

        public static final class Mod3 {
            public static final double angleOffSet = 168.22;

        }

    }
    
}
