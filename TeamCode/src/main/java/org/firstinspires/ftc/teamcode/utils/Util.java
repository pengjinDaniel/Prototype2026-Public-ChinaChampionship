package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.angleUnit;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.distanceUnit;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.farGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.nearGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.redGoalPose;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.opencv.core.Mat;

public class Util {
    public static boolean epsilonEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }

    public static Pose2D visionPoseToDWPose(Pose3D pose3D) {
        return new Pose2D(distanceUnit, Units.metersToInches(pose3D.getPosition().y) + 72,
                -Units.metersToInches(pose3D.getPosition().x) + 72, angleUnit,
                pose3D.getOrientation().getYaw() / 180 * Math.PI - Math.PI / 2);
    }

    public static double poseDistance(Pose2D poseA, Pose2D poseB) {
        return Math.sqrt(Math.pow(poseA.getX(distanceUnit) - poseB.getX(distanceUnit), 2)
                + Math.pow(poseA.getY(distanceUnit) - poseB.getY(distanceUnit), 2));
    }

    public static double adjustRange(double rad) {
        if (rad > Math.PI) rad -= Math.ceil((rad - Math.PI) / (2 * Math.PI)) * 2 * Math.PI;
        if (rad < -Math.PI) rad += Math.ceil((-Math.PI - rad) / (2 * Math.PI)) * 2 * Math.PI;
        return rad;
    }

    public static double adjustRange(double rad, double epsilon) {
        if (rad > Math.PI + epsilon) rad -= Math.ceil((rad - Math.PI) / (2 * Math.PI)) * 2 * Math.PI;
        if (rad < -Math.PI - epsilon) rad += Math.ceil((-Math.PI - rad) / (2 * Math.PI)) * 2 * Math.PI;
        return rad;
    }

    public static PolarVector goalInRobotSys(Pose2D drivePose, Drive.Alliance alliance) {
        Pose2D goalPose = alliance == Drive.Alliance.BLUE? blueGoalPose: redGoalPose;
        double cos_h = Math.cos(drivePose.getHeading(AngleUnit.RADIANS));
        double sin_h = Math.sin(drivePose.getHeading(AngleUnit.RADIANS));
        double dx = goalPose.getX(distanceUnit) - drivePose.getX(distanceUnit);
        double dy = goalPose.getY(distanceUnit) - drivePose.getY(distanceUnit);
        return new PolarVector(DistanceUnit.INCH, dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h);
    }

    public static double getShooterVelocity(PolarVector goalToRobot) {
        double distanceToGoal = goalToRobot.magnitude;
        if (distanceToGoal != -1) {
            double normalizedDistance = (distanceToGoal - nearGoalDistance) / (farGoalDistance - nearGoalDistance);

            double nonlinearFactor = 1.0 + 0.04 * normalizedDistance;

            double finalVelocity = (ShooterConstants.slowVelocity
                    + (ShooterConstants.fastVelocity - ShooterConstants.slowVelocity)
                    * normalizedDistance) * nonlinearFactor;

            return 20 * Math.ceil(finalVelocity / 20);
        }
        return 0;
    }
}
