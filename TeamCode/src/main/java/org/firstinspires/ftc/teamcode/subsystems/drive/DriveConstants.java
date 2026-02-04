package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Util;

@Config
public class DriveConstants {
    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";
    public static String odName = "od";

    public static double xPoseDW = Units.mmToInches(-166.2), yPoseDW = Units.mmToInches(24.5);

    public static double strafingBalance = 1.1;
    public static double headingEpsilon = 0.1;
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;

    public static double forwardVelocity = 81.663, strafeVelocity = 63.959;
    public static double forwardAcceleration = -31.129, strafeAcceleration = -55.733;

    public static double kP_brakeXY = 0.02;
    public static double kP_brakeH = -0.8;
    public static double kP_alignH = -1;
    public static double kI_alignH = 0;
    public static double kD_alignH = -0.01;
    public static double kF_alignH = -0.03;

    public static double xNearPoseRed = 80, yNearPoseRed = 91;
    public static double xNearPoseBlue = 57, yNearPoseBlue = 92;
    public static double xFarPoseRed = 137, yFarPoseRed = 140;
    public static double xFarPoseBlue = 63, yFarPoseBlue = 13;
    public static Pose2D blueGoalPose = new Pose2D(distanceUnit, 5.0, 135.7, angleUnit, 0);
    public static Pose2D redGoalPose = new Pose2D(distanceUnit, 135.2, 135.7, angleUnit, 0);


    //86 24, 60 86
    public static double nearGoalDistance =
            Util.poseDistance(
                    new Pose2D(DistanceUnit.INCH,
                            redGoalPose.getX(distanceUnit),
                            redGoalPose.getY(distanceUnit), AngleUnit.RADIANS,
                            0
                    ),
                    new Pose2D(DistanceUnit.INCH,
                            xNearPoseRed,
                            yNearPoseRed,
                            AngleUnit.RADIANS,
                            0
                    )
            );

    public static double farGoalDistance =
            Util.poseDistance(
                    new Pose2D(DistanceUnit.INCH,
                            redGoalPose.getX(distanceUnit),
                            redGoalPose.getY(distanceUnit),
                            AngleUnit.RADIANS,
                            0
                    ),
                    new Pose2D(
                            DistanceUnit.INCH,
                            xFarPoseRed,
                            yFarPoseRed,
                            AngleUnit.RADIANS,
                            0
                    )
            );

    public static double nearFlyTime = 0;
    public static double farFlyTime = 0;

    public static Pose2D autoEndPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
}
