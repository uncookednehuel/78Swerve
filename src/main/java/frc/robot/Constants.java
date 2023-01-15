package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.classes.COTSFalconSwerveConstants;
import frc.robot.classes.SwerveModuleConstants;

public final class Constants {

    public static final class Swerve {

        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);
            // COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        //MK4 drivetrain constants
        // public static final double trackWidth = Units.inchesToMeters(22);
        // public static final double wheelBase = Units.inchesToMeters(22);
        //MK4i drivetrain constants
        public static final double trackWidth = Units.inchesToMeters(23);
        public static final double wheelBase = Units.inchesToMeters(23);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(269.29); //MK4
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(10.28); //MK4i
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(292.93); //MK4
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.24); //MK4i
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(17.31); //MK4
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(179.64); //MK4i
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(344.70); //MK4
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(168.04); //MK4i
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final int pigeonIMU = 0;
    public static final String photonCam = "photonvision";
    
    //  CONTROLLERS
    public static final int driverController = 0;

    //#region KINEMATICS

    //keep in mind that the locations for the modules must be relative to the center of the robot. Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
    //order for swerve modules is always left upper, right upper, left lower, right lower
    public static final Translation2d wheelLU = new Translation2d(0.5588, 0.5588);
    public static final Translation2d wheelRU = new Translation2d(0.5588, -0.5588);
    public static final Translation2d wheelLD = new Translation2d(-0.5588, 0.5588);
    public static final Translation2d wheelRD = new Translation2d(-0.5588, -0.5588);

    public static final double offsetLU = Math.toRadians(269.20) * -1; //put real offsets from smartdashboard
    public static final double offsetRU = Math.toRadians(292.84) * -1;
    public static final double offsetLD = Math.toRadians(15.22) * -1;
    public static final double offsetRD = Math.toRadians(344.35) * -1;

    public static final double maxVoltage = 12.0;
    public static final double DRIVE_GEAR_RATIO = 1 / 6.75;
    public static final double STEER_GEAR_RATIO = 1 / 12.8;
    public static final double WHEEL_CIRCUMFERENCE = 0.09652; //meters
    //6380 is falcon FX max rpm, / 60 (to get revolutions per second), * gear ratio (to wheel rps), * wheel circumference
    public static final double maxSpeed = 6380.0 / 60.0 * DRIVE_GEAR_RATIO * 0.1016 * Math.PI; //meters per second
    //#endregion
    
     //#region PATH FOLLOWING
     public static final double maxVel = 1; //in meters per second
     public static final double maxAcc = 1; //in meters per second squared
     public static final double maxRotVel = 6.28; //6.28 is 1 rotation (it is 2xPI but I don't know why exactly)
     public static final double maxRotAcc = 3.14; //3.14 is half a rotation
     public static final double xErrVel = 1; // I don't know what "additional meter per second in the x direction for every meter of error in the x direction" means in the docs
     public static final double yErrVel = 1;
 
     public static final double kI = 0;
     public static final double kD = 0;
     //#endregion
}
