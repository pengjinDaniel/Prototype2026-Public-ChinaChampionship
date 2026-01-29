package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String pitchServoName = "pitchServo";

    public static double shooterEpsilon = 60;

    public static double highPose = 0.85;
    public static double middlePose = 0.6;
    public static double lowPose = 0.0;

    /**
     * In Ticks Per Second
     */
    public static double stopVelocity = 0;
    public static double fastVelocity = 1460;
    public static double slowVelocity = 1100;
    public static double kP = -140.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = -14.0;


    public static double kP1 = -140.0;
    public static double kI1 = 0.0;
    public static double kD1 = 0.0;
    public static double kF1 = -14.0;
}
