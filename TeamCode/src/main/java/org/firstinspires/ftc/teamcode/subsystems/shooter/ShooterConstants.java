package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String pitchServoName = "pitchServo";

    public static double shooterEpsilon = 20;

    public static double highPose = 1;
    public static double middlePose = 0.5;
    public static double lowPose = 0;

    /**
     * In Ticks Per Second
     */
    public static double stopVelocity = 0;
    public static double fastVelocity = 1520; // 1520;
    public static double slowVelocity = 1300; // 1300;
    public static double releaseVelocity = 1000;
    public static double slowPower = 0.8;
    public static double fastPower = 0.95;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(140, 0, 0, 0);
}
