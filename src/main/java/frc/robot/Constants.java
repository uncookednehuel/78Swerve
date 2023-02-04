package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.classes.COTSFalconSwerveConstants;
import frc.robot.classes.SwerveModuleConstants;

public final class Constants {

    /** A class within Constants that contains most of the swerve constants */
    public static final class Swerve {

        public static final COTSFalconSwerveConstants CHOSEN_MODULE =
        // COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        // MK4 drivetrain constants
        public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        public static final double WHEEL_BASE = Units.inchesToMeters(22);
        // Ant Man drivetrain constants
        // public static final double TRACK_WIDTH = Units.inchesToMeters(18.75);
        // public static final double WHEEL_BASE = Units.inchesToMeters(18.75);
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;
        public static final boolean CANCODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;
        public static final double ANGLE_KF = CHOSEN_MODULE.angleKF;
        public static final double ANGLE_MAX_ERR = CHOSEN_MODULE.angleMaxErr;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.05; // TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double DRIVE_KS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(269.29); //MK4
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(10.10); // ant man
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(292.93); //MK4
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.75); // ant man
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(17.31); //MK4
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(178.68); // ant man
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(344.70); //MK4
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(166.64); // ant man
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final int PIGEON_IMU = 0;

    // CONTROL
    public static final int DRIVE_CONTROLLER = 0;
    public static final double DPAD_VEL = 1; // max meters per second (with RT down)

    // #region KINEMATICS

    // keep in mind that the locations for the modules must be relative to the
    // center of the robot. Positive x values represent moving toward the front of
    // the robot whereas positive y values represent moving toward the left of the
    // robot.
    // order for swerve modules is always left upper, right upper, left lower, right
    // lower

    // 6380 is falcon FX max rpm, / 60 (to get revolutions per second), * gear ratio
    // (to wheel rps), * wheel circumference
    public static final double MAX_SPEED = 6380.0 / 60.0 * Swerve.DRIVE_GEAR_RATIO * Units.inchesToMeters(Swerve.WHEEL_CIRCUMFERENCE) * Math.PI;
    // #endregion

    // #region PATH FOLLOWING
    public static final double PATH_MAX_VEL = 1; // in meters per second
    public static final double PATH_MAX_ACC = 1; // in meters per second squared
    public static final double PATH_ROT_MAX_VEL = 6.28; // 6.28 is 1 rotation (it is 2xPI but I don't know why exactly)
    public static final double PATH_ROT_MAX_ACC = 3.14; // 3.14 is half a rotation
    public static final double X_ERROR_VEL = 1; // I don't know what "additional meter per second in the x direction for
                                                // every meter of error in the x direction" means in the docs
    public static final double Y_ERROR_VEL = 1;

    public static final double TRAJECTORY_KI = 0;
    public static final double TRAJECTORY_KD = 0;
    // #endregion
}
