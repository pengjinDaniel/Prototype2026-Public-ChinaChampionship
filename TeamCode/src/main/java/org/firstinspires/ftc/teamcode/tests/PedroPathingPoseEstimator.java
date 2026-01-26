package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;

@Config
@TeleOp(name = "PedroPathingPoseEstimator")
public class PedroPathingPoseEstimator extends OpMode {
    private Follower follower;
    public static double startX, startY, startHeading;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeading)));
    }

    @Override
    public void loop() {
        follower.update();
        TelemetryPacket packet = new TelemetryPacket();
        Pose pose = follower.getPose();

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        packet.put("x", x);
        packet.put("y", y);
        packet.put("heading (deg)", Math.toDegrees(heading));

        packet.fieldOverlay().setFill("blue");
        packet.fieldOverlay().fillCircle(x, y, 2.0);

        double headingLength = 5.0;
        double hx = x + Math.cos(heading) * headingLength;
        double hy = y + Math.sin(heading) * headingLength;

        packet.fieldOverlay().setStroke("red");
        packet.fieldOverlay().strokeLine(x, y, hx, hy);

        dashboard.sendTelemetryPacket(packet);
    }
}
