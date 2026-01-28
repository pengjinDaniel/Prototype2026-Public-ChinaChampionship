package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.teamcode.utils.Units;

public class TurretConstants {
    public static String turretMotorName = "turretMotor";
    public static double initPos = 0.0;
    public static double kP = 0.0007;
    public static double kI = 0;
    public static double kD = 0.000012;
    public static double kF = 0;
    public static double rangeEpsilon = Units.degreesToRadians(40);
    public static double alignEpsilon = 100;

    public static double offset = Math.PI / 4;

    public static double offsetToRobot = Units.mmToInches(29.286);
}
