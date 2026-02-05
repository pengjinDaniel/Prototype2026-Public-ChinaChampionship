package org.firstinspires.ftc.teamcode.subsystems.drive;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.farFlyTime;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.farGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_brakeH;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_brakeXY;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.nearFlyTime;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.nearGoalDistance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.speedLimit;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.xPoseDW;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.yPoseDW;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

@Config
public class Drive extends SubsystemBase {
    public final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private final GoBildaPinpointDriver od;
    private double yawOffset;// mm
    private final Alliance alliance;
    private DriveState driveState;
    private Telemetry telemetry;
    private boolean aligned;
    private boolean scoring;

    Pose2D lastPose;

    private static final int MEDIAN_FILTER_SIZE = 10;
    private final Queue<Double> velXBuffer = new LinkedList<>();
    private final Queue<Double> velYBuffer = new LinkedList<>();
    private final Queue<Double> headingVelBuffer = new LinkedList<>();
    private final double[] velXArray = new double[MEDIAN_FILTER_SIZE];
    private final double[] velYArray = new double[MEDIAN_FILTER_SIZE];
    private final double[] headingVelArray = new double[MEDIAN_FILTER_SIZE];

    // 存储滤波后的速度值，在 periodic() 中更新
    private double filteredVelX = 0;
    private double filteredVelY = 0;
    private double filteredHeadingVel = 0;

    public enum DriveState {
        STOP,
        TELEOP;
        DriveState() {}
    }

    public enum Alliance {
        RED,
        BLUE;

        Alliance() {}
    }

    public Drive(final HardwareMap hardwareMap, Alliance alliance) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, DriveConstants.leftFrontMotorName);
        leftBackMotor = hardwareMap.get(DcMotor.class, DriveConstants.leftBackMotorName);
        rightFrontMotor = hardwareMap.get(DcMotor.class, DriveConstants.rightFrontMotorName);
        rightBackMotor = hardwareMap.get(DcMotor.class, DriveConstants.rightBackMotorName);
        od = hardwareMap.get(GoBildaPinpointDriver.class, DriveConstants.odName);
        driveState = DriveState.STOP;
        this.alliance = alliance;

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        od.resetPosAndIMU();
        od.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        od.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        od.setOffsets(xPoseDW, yPoseDW, DriveConstants.distanceUnit);

        od.setPosition(DriveConstants.autoEndPose);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPose = new Pose2D(DriveConstants.distanceUnit, 0, 0, DriveConstants.angleUnit, 0);
        aligned = false;
        this.scoring = false;

        initializeFilterBuffers();
    }

    private void initializeFilterBuffers() {
        for (int i = 0; i < MEDIAN_FILTER_SIZE; i++) {
            velXBuffer.add(0.0);
            velYBuffer.add(0.0);
            headingVelBuffer.add(0.0);
        }
    }

    private double applyMedianFilter(Queue<Double> buffer, double newValue, double[] array) {
        buffer.poll();
        buffer.add(newValue);

        int index = 0;
        for (Double value : buffer) {
            array[index++] = value;
        }

        Arrays.sort(array);

        return array[MEDIAN_FILTER_SIZE / 2];
    }

    // 在 periodic() 中调用，更新滤波值
    private void updateFilteredVelocities() {
        double rawVelX = od.getVelX(DriveConstants.distanceUnit);
        double rawVelY = od.getVelY(DriveConstants.distanceUnit);
        double rawHeadingVel = od.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        filteredVelX = applyMedianFilter(velXBuffer, rawVelX, velXArray);
        filteredVelY = applyMedianFilter(velYBuffer, rawVelY, velYArray);
        filteredHeadingVel = applyMedianFilter(headingVelBuffer, rawHeadingVel, headingVelArray);
    }

    public double getFilteredVelX() {
        return filteredVelX;
    }

    public double getFilteredVelY() {
        return filteredVelY;
    }

    public double getFilteredHeadingVelocity(org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit unit) {
        // 滤波值已在 periodic() 中以 RADIANS 单位计算
        return filteredHeadingVel;
    }

    public boolean isMoving() {
        return Math.abs(getFilteredHeadingVelocity(UnnormalizedAngleUnit.RADIANS)) > DriveConstants.epsilonStopH ||
               Math.abs(getFilteredVelX()) > DriveConstants.epsilonStopXY ||
               Math.abs(getFilteredVelY()) > DriveConstants.epsilonStopXY;
    }

    public void setScoring(boolean scoring) {
        this.scoring = scoring;
    }

    public void stop() {
        moveRobot(0, 0, 0);
    }

    public void reset(double heading) {
        yawOffset = od.getPosition().getHeading(DriveConstants.angleUnit) + heading;
    }

    public void setAligned(boolean aligned) {
        this.aligned = aligned;
    }

    public boolean getAligned() {
        return aligned;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setDriveState(DriveState driveState) {
        this.driveState = driveState;
    }

    public void moveRobotFieldRelative(double forward, double fun, double turn) {

        double botHeading = od.getPosition().getHeading(DriveConstants.angleUnit) - yawOffset;
        // Rotate the movement direction counter to the bot's rotation\\
        double rotX = fun * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = fun * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        rotX = rotX * strafingBalance; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        double limitFactor = scoring? speedLimit : 1;
        leftFrontMotor.setPower(leftFrontPower * limitFactor);
        leftBackMotor.setPower(leftBackPower * limitFactor);
        rightFrontMotor.setPower(rightFrontPower * limitFactor);
        rightBackMotor.setPower(rightBackPower * limitFactor);
    }

    public void moveRobot(double forward, double fun, double turn) {
        double rotX = fun * strafingBalance; // Counteract imperfect strafing
        double rotY = forward * 1;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        double limitFactor = scoring? speedLimit : 1;
        leftFrontMotor.setPower(leftFrontPower * limitFactor);
        leftBackMotor.setPower(leftBackPower * limitFactor);
        rightFrontMotor.setPower(rightFrontPower * limitFactor);
        rightBackMotor.setPower(rightBackPower * limitFactor);
    }

    public Pose2D getPose() {
        return od.getPosition();
    }

    public double getYawOffset() {return yawOffset;}

    private void applyBrake() {
        Pose2D p = getPose();

        double errorX = lastPose.getX(DriveConstants.distanceUnit) - p.getX(DriveConstants.distanceUnit);
        double errorY = lastPose.getY(DriveConstants.distanceUnit) - p.getY(DriveConstants.distanceUnit);
        double errorH = angleWrap(lastPose.getHeading(DriveConstants.angleUnit) - p.getHeading(DriveConstants.angleUnit));

        double forward = errorY * kP_brakeXY;
        double strafe = errorX * kP_brakeXY;
        double turn = errorH * kP_brakeH;

        forward = clip(forward, -1, 1);
        strafe = clip(strafe, -1, 1);
        turn = clip(turn, -1, 1);

        moveRobotFieldRelative(forward, strafe, turn);
    }

    public double getFlyTime(Alliance alliance) {
        return nearFlyTime + (farFlyTime - nearFlyTime) / (farGoalDistance - nearGoalDistance)
                * (Util.goalInTurretSys(getPose(), alliance).getMagnitude() - nearGoalDistance);
    }

    public Pose2D getExpectedPose(Alliance alliance) {
        double velX = getFilteredVelX();
        double velY = getFilteredVelY();
        double headingVel = getFilteredHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        if (!isMoving()) {
            velX = 0;
            velY = 0;
            headingVel = 0;
        }

        double flyTime = getFlyTime(alliance);

        return new Pose2D(DriveConstants.distanceUnit,
                getPose().getX(DriveConstants.distanceUnit) + velX * flyTime,
                getPose().getY(DriveConstants.distanceUnit) + velY * flyTime,
                DriveConstants.angleUnit,
                getPose().getHeading(DriveConstants.angleUnit));
    }

    public void setPose(Pose2D pose) {
        od.setPosition(pose);
    }

    public void setYawOffset(double yawOffset) {
        this.yawOffset = yawOffset;
    }

    @Override
    public void periodic() {
        od.update();
        updateFilteredVelocities();
//        if (driveState == DriveState.STOP) {
//            applyBrake();
//        }
        lastPose = getPose();
    }
}