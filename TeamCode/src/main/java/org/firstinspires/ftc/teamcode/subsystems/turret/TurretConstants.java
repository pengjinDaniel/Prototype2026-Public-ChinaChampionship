package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.teamcode.utils.Units;

public class TurretConstants {
    public static String turretMotorName = "turretMotor";
    public static double initPos = 0.0;
    public static double kP = 0.00075;
    public static double kI = 0;
    public static double kD = 0.00001;
    public static double kF = 0;
//    public static double kPMoving = 0.001;
//    public static double kIMoving = 0.05;
//    public static double kDMoving = 0.00002;
//    public static double kFMoving = 0;
    public static double rangeEpsilon = Units.degreesToRadians(40);
    public static double alignEpsilon = 100;

    public static double offset = Math.PI / 4;

    public static double offsetToRobot = Units.mmToInches(29.286);
}
