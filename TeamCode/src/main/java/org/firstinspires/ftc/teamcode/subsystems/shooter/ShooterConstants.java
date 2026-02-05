package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String pitchServoName = "pitchServo";

    public static double shooterEpsilon = 40;

    // Servo
    public static double highPose = 0.9;
    public static double middlePose = 0.6;
    public static double lowPose = 0.00;

    public static double near1Pose = 0.2;
    public static double near2Pose = 0.3;
    public static double near3Pose = 0.45;
    public static double near4Pose = 0.5;
    public static double near5Pose = 0.63;
    public static double far1Pose = 0.82;
    public static double far2Pose = 0.82;

    /**
     * In Ticks Per Second
     */
    // Motor
    public static double stopVelocity = 0;
    public static double maxVelocity = 1520;
    public static double fastVelocity = 1420;
    public static double slowLimitVelocity = 1210;
    public static double slowVelocity = 1130;
    public static double minVelocity = 940;
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0005;
}
