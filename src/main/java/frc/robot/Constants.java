package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double SWERVE_STEERING_RATIO = (150.0 / 7.0);
        public static final double kPTurning = 0.2;

    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(20.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(20.75);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // FIXME: patch these motor IDs up to match the Swervie 2022 configuration
        public static final int kFrontLeftDriveMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 18;
        public static final int kBackLeftDriveMotorPort = 10;
        public static final int kBackRightDriveMotorPort = 19;

        public static final int kFrontLeftTurningMotorPort = 16;
        public static final int kFrontRightTurningMotorPort = 15;
        public static final int kBackLeftTurningMotorPort = 21;   
        public static final int kBackRightTurningMotorPort = 12;

        public static final int gyroPort = 60;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        // CANCoder IDs
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 20;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 17;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 14;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffset  = 0.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffset = 0.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffset   = 0.0;
        public static final double kBackRightDriveAbsoluteEncoderOffset  = 0.0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0292;   // MK4i 16.5 FPS => 5.0292 m/s
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6;

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0, 0), // Translation constants 
            new PIDConstants(5.0, 0, 0), // Rotation constants 
            kPhysicalMaxSpeedMetersPerSecond, 
            new Translation2d(kWheelBase / 2, kTrackWidth / 2).getNorm(), // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig()
        );

        public static final double DRIVE_DIVIDER_NORMAL = 2.0;
        public static final double DRIVE_DIVIDER_TURBO = 1.0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_ROLLER_MOTOR = 58;
        public static final int INTAKE_ANGLE_MOTOR = 59;
        public static final int INTAKE_ANGLE_MOTOR_FOLLOWER = 62;
        public static final int INTAKE_ANGLE_CANCODER = 61;
        public static final int TALON_TIMEOUT_MS = 5000;
        public static final double INTAKE_ANGLE_STOWED = 0;
        public static final double INTAKE_FLOOR_PICKUP = 189;
        public static final double INTAKE_HALF = 94.5;
        public static final double INTAKE_ANGLE_TOLERANCE = 7;
        public static final double INTAKE_ANGLE_MOTOR_ACCELERATION = 3000;
        public static final double INTAKE_ANGLE_MOTOR_CRUISE = 7000;
        public static final double INTAKE_ANGLE_MOTOR_KP = 0.5;
        public static final double INTAKE_ANGLE_MOTOR_KI = 0;
        public static final double INTAKE_ANGLE_MOTOR_KD = 0.06;
        public static final double INTAKE_ANGLE_MOTOR_KF = 0;
        public static final double INTAKE_ANGLE_MOTOR_MAX = 191;
        public static final double INTAKE_ANGLE_MOTOR_MIN = 0;

    }

    public static final class ShooterConstants{
        public static final int SHOOTER_MOTOR_TOP = 3;
        public static final int SHOOTER_MOTOR_BOTTOM = 4;
        public static final double SHOOTER_SPEED_TOLERANCE = 100.0;
        public static final double SHOOTER_SPEED_SPEAKER = 4000; //5000;
        public static final double SHOOTER_SPEED_PODIUM = 6000;
        public static final double SHOOTER_SPEED_WING = 5000;
        public static final double SHOOTER_SPEED_AMP = 2000;
    }

    public static final class ShooterPivotConstants
    {
        public static final int SHOOTER_PIVOT_MOTOR = 36;
        public static final int SHOOTER_PIVOT_MOTOR_FOLLOWER = 35;
        public static final int SHOOTER_PIVOT_CANCODER = 37;
        //fill out position values later
        public static final double SHOOTER_PIVOT_START = 10;
        public static final double SHOOTER_PIVOT_CLOSE = 22.0;
        public static final double SHOOTER_PIVOT_PODIUM = 45;
        public static final double SHOOTER_PIVOT_WING = 53;
        public static final double SHOOTER_PIVOT_SLOT1 = 60;
        public static final double SHOOTER_PIVOT_HIGH = 70;

        public static final double SHOOTER_PIVOT_AMP = 100;
        public static final double SHOOTER_PIVOT_TRAP = 90;
        public static final double SHOOTER_PIVOT_MAX = 210; // max travel
        public static final double SHOOTER_PIVOT_MIN = 10;
        public static final double SHOOTER_PIVOT_ACCELERATION = 500;
        public static final double SHOOTER_PIVOT_CRUISE = 500;
        public static final double SHOOTER_PIVOT_0_KP = 1.4;
        public static final double SHOOTER_PIVOT_0_KI = 0.009;
        public static final double SHOOTER_PIVOT_0_KD = 0.0;
        public static final double SHOOTER_PIVOT_0_KF = 0.2;
        // slot 1 for amp/trap
        public static final double SHOOTER_PIVOT_1_KP = 1.2;
        public static final double SHOOTER_PIVOT_1_KI = 0.004;
        public static final double SHOOTER_PIVOT_1_KD = 0.0;
        public static final double SHOOTER_PIVOT_1_KF = 0;

    }

    public static final class ElevatorConstants{
        public static final int ELEVATOR_MOTOR = 42;
        public static final double ELEVATOR_POSITION_TOLERANCE = 0.5;
        //fill out position values later
        public static final double ELEVATOR_START = 0;
        public static final double ELEVATOR_WING = -7.9;
        public static final double ELEVATOR_PODIUM = -9.5;
        public static final double ELEVATOR_CLOSE = -7.9;
        public static final double ELEVATOR_AMP_START = -5.66;
        public static final double ELEVATOR_AMP = -12;
        public static final double ELEVATOR_AMP_MAX = -12.5;
        public static final double ELEVATOR_TRAP = 0;
    }

     public static final class ClimbConstants {
        public static final int CLIMB_MOTOR = 24;
        public static final int CLIMB_MOTOR_FOLLOWER = 25;
        public static final int CLIMB_MAX = 0;
        public static final int CLIMB_MIN = 0;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.15;
    }
}
