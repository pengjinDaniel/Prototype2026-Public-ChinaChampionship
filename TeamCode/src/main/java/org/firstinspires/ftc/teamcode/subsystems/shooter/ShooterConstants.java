package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String brakeServoName = "brakeServo";

    public static double shooterEpsilon = 20;

    public static double brakePose = 0.23;
    public static double releasePose = 0.5;

    /**
     * In Ticks Per Second
     */
    public static double stopVelocity = 300;
    public static double fastVelocity = 1300; // 1520;
    public static double slowVelocity = 920; // 1300;
    public static double releaseVelocity = 1000;
    public static double slowPower = 0.8;
    public static double fastPower = 0.95;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(140, 0, 0, 0);
}
