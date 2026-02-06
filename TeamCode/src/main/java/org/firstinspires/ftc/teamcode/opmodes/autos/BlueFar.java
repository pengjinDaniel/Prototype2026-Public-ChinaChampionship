package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterAlignCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Autonomous(name = "Blue Far", group = "Auto")
public class BlueFar extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Drive.Alliance alliance;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

    public FtcDashboard dashboard;

    public Command transitShootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> vision.autoCalibrate(follower, turret)),
                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new TransitCommand(shooter, transit, intake)
                                .andThen(new WaitCommand(200))
                                .andThen(new ShootCommand(intake, shooter))
                ),
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP))
        );
    }

    public Command intakeCommand(PathChain path) {
        return new ParallelRaceGroup(
                new AutoDriveCommand(follower, path),
                new IntakeCommand(intake, transit)
        ).andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));
    }

    public Command cycleCommand() {
        return new SequentialCommandGroup(
                intakeCommand(Path4),
                new ParallelRaceGroup(
                        new WaitCommand(1000),
                        new IntakeCommand(intake, transit)
                ),
                intakeCommand(Path5),
                transitShootCommand()
        );
    }

    @Override
    public void initialize() {
        this.follower = Constants.createFollower(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.transit = new Transit(hardwareMap, false);
        this.turret = new Turret(hardwareMap);
        this.vision = new Vision(hardwareMap);
        this.alliance = Drive.Alliance.BLUE;

        this.dashboard = FtcDashboard.getInstance();

        follower.setStartingPose(new Pose(55.789, 7.527, Math.toRadians(180)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.789, 7.527),

                                new Pose(55.804, 11.079)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.804, 11.079),

                                new Pose(13.021, 8.129)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.021, 8.129),

                                new Pose(55.894, 11.080)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.894, 11.080),

                                new Pose(12.922, 10.941)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.922, 10.941),

                                new Pose(55.929, 11.204)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.929, 11.204),

                                new Pose(37.245, 11.024)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        schedule(
                new ParallelCommandGroup(
                        new TurretAlignCommand(follower, turret, alliance, vision),
                        new ShooterAlignCommand(follower, shooter, transit, alliance),
                        new SequentialCommandGroup(
                                new AutoDriveCommand(follower, Path1),
                                new WaitCommand(1500),
                                transitShootCommand(),
                                intakeCommand(Path2),
                                new ParallelRaceGroup(
                                        new WaitCommand(1000),
                                        new IntakeCommand(intake, transit)
                                ),
                                intakeCommand(Path3),
                                transitShootCommand(),
                                cycleCommand(),
                                cycleCommand(),
                                new AutoDriveCommand(follower, Path6)
                        )
                )
        );
    }

    private void printPose() {
        TelemetryPacket packet = new TelemetryPacket();
        Pose pose = follower.getPose();

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        double xDraw = -72 + x;
        double yDraw = -72 + y;

        double headingDraw = heading + Math.PI;

        headingDraw = Math.atan2(Math.sin(headingDraw), Math.cos(headingDraw));

        packet.put("x(inch)", x);
        packet.put("y(inch)", y);
        packet.put("heading(deg)", Math.toDegrees(heading));

        packet.fieldOverlay().setFill("blue");
        packet.fieldOverlay().fillCircle(xDraw, yDraw, 2.0);

        double headingLength = 5.0;
        double hx = xDraw + Math.cos(headingDraw) * headingLength;
        double hy = yDraw + Math.sin(headingDraw) * headingLength;

        packet.fieldOverlay().setStroke("red");
        packet.fieldOverlay().strokeLine(xDraw, yDraw, hx, hy);

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Shooter at Setpoint: ", shooter.isShooterAtSetPoint());
        telemetry.update();

        printPose();
    }
}
