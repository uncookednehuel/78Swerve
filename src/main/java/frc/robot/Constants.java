package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    //  MOTOR IDs
    //format: drive(Side L left or R right, Side U up or D down, Type D drive or H heading)
    public static final int driveLUD = 1;
    public static final int driveLUH = 2;

    public static final int driveRUD = 3;
    public static final int driveRUH = 4;
    
    public static final int driveLDD = 5;
    public static final int driveLDH = 6;

    public static final int driveRDD = 7;
    public static final int driveRDH = 8;

    //  SENSORS
    public static final int encLU = 9;
    public static final int encRU = 10;
    public static final int encLD = 11;
    public static final int encRD = 12;

    public static final int pigeonIMU = 0;
    public static final String photonCam = "photonvision";
    
    //  CONTROLLERS
    public static final int driverController = 0;

    //#region

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
        public static final double AZIMUTH_MOTOR_KP = 0.2;
        ;
        public static final double AZIMUTH_MOTOR_KI = 0;
        public static final double AZIMUTH_MOTOR_KD = 0;
        public static final double AZIMUTH_MOTOR_KF = 0;

        /* Drive Motor PID Values */
        public static final double driveMotorKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveMotorKI = 0.0;
        public static final double driveMotorKD = 0.0;
        public static final double driveMotorKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveMotorKS = (0.32 / 12);
        public static final double driveMotorKV = (1.51 / 12);
        public static final double driveMotorKA = (0.27 / 12);
    //#endregion

    //#region KINEMATICS

    //keep in mind that the locations for the modules must be relative to the center of the robot. Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
    //order for swerve modules is always left upper, right upper, left lower, right lower
    public static final Translation2d wheelLU = new Translation2d(0.5588, 0.5588);
    public static final Translation2d wheelRU = new Translation2d(0.5588, -0.5588);
    public static final Translation2d wheelLD = new Translation2d(-0.5588, 0.5588);
    public static final Translation2d wheelRD = new Translation2d(-0.5588, -0.5588);
    //OFFSETS
    public static final double offsetLU = Math.toRadians(269.20) * -1; //put real offsets from smartdashboard
    public static final double offsetRU = Math.toRadians(292.84) * -1;
    public static final double offsetLD = Math.toRadians(15.22) * -1;
    public static final double offsetRD = Math.toRadians(344.35) * -1;

    //Module constants
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static final double angleRatio = 12.8 / 1.0;
    public static final double driveRatio = 6.86 / 1.0;

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
