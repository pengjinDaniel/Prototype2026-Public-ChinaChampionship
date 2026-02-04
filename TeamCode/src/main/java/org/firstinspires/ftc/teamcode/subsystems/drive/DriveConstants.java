package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.ArrayList;
import java.util.List;

@Config
public class DriveConstants {
    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";
    public static String odName = "od";

    public static double xPoseDW = Units.mmToInches(-166.2), yPoseDW = Units.mmToInches(24.5);

    public static double strafingBalance = 1;
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;

    public static double forwardVelocity = 81.663, strafeVelocity = 63.959;
    public static double forwardAcceleration = -31.129, strafeAcceleration = -55.733;

    public static double kP_brakeXY = 0.02;
    public static double kP_brakeH = -0.8;
    public static double epsilonStopXY = 5;
    public static double epsilonStopH = 0.5;
    public static double speedLimit = 1;

    public static double xNearPoseRed = 80, yNearPoseRed = 91;
//    public static double xNearPoseBlue = 64, yNearPoseBlue = 91;
    public static double xFarPoseRed = 63, yFarPoseRed = 13;
//    public static double xFarPoseBlue = 81, yFarPoseBlue = 13;
    public static Pose2D blueGoalPose = new Pose2D(distanceUnit, 7, 134.7, angleUnit, 0);
    public static Pose2D redGoalPose = new Pose2D(distanceUnit, 137, 134.7, angleUnit, 0);


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
    //105 110
    public static double near1Distance = 40.423;
    //100 110
    public static double near2Distance = 44.487;
    //95 100
    public static double near3Distance = 54.479;
    //80 90
    public static double near4Distance = 72.439;
    //72 72
    public static double near5Distance = 90.311;
    //70 18
    public static double far1Distance = 134.566;
    //63 13
    public static double far2Distance = 142.433;
    public static ArrayList<Double> poseList = new ArrayList<>(List.of(near1Distance, near2Distance,
            near3Distance, near4Distance, near5Distance, far1Distance, far2Distance));

    public static double nearFlyTime = 0.7;
    public static double farFlyTime = 0.7;

    public static Pose2D autoEndPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
}
