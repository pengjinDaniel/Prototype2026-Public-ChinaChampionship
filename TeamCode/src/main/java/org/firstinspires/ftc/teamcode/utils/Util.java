package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.angleUnit;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.distanceUnit;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.farGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.nearGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.redGoalPose;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.fastVelocity;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.slowVelocity;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

public class Util {
    public static Pose Pose2DToPose(Pose2D pose2D) {
        return new Pose(pose2D.getX(DriveConstants.distanceUnit),
                pose2D.getY(DriveConstants.distanceUnit),
                pose2D.getHeading(DriveConstants.angleUnit));
    }

    public static Pose2D PoseToPose2D(Pose pose) {
        return new Pose2D(distanceUnit, pose.getX(), pose.getY(),
                angleUnit, pose.getHeading());
    }

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

    public static Pose2D drivePoseToTurretPose(Pose2D drivePose) {
        return new Pose2D(
                distanceUnit,
                drivePose.getX(distanceUnit) + Math.cos(drivePose.getHeading(angleUnit)) * TurretConstants.offsetToRobot,
                drivePose.getY(distanceUnit) + Math.sin(drivePose.getHeading(angleUnit)) * TurretConstants.offsetToRobot,
                angleUnit,
                drivePose.getHeading(angleUnit)
        );
    }

    public static PolarVector goalInTurretSys(Pose2D drivePose, Drive.Alliance alliance) {
        Pose2D goalPose = alliance == Drive.Alliance.BLUE ? blueGoalPose : redGoalPose;
        Pose2D turretPose = drivePoseToTurretPose(drivePose);
        double cos_h = Math.cos(drivePose.getHeading(AngleUnit.RADIANS));
        double sin_h = Math.sin(drivePose.getHeading(AngleUnit.RADIANS));
        double dx = goalPose.getX(distanceUnit) - turretPose.getX(distanceUnit);
        double dy = goalPose.getY(distanceUnit) - turretPose.getY(distanceUnit);
        return new PolarVector(DistanceUnit.INCH, dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h);
    }

    public static double getShooterVelocity(PolarVector goalToTurret) {
        double distanceToGoal = goalToTurret.magnitude;
        double k = (fastVelocity - slowVelocity) / (farGoalDistance - nearGoalDistance);
        return slowVelocity + k * (distanceToGoal - nearGoalDistance);
    }
}
