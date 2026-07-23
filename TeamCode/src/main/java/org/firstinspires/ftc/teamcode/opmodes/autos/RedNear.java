package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LedWinkCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterAlignCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.led.Led;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Autonomous(name = "Red Near", group = "Auto")
public class RedNear extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Led led;
    private Drive.Alliance alliance;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8,
            Path9, Path10, Path11, Path12, Path13, Path14, Path15;

    public Command transitShootCommand() {
        return new SequentialCommandGroup(
                new TransitCommand(shooter, transit, intake, 800),
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)),
                new ConditionalCommand(
                        new LedWinkCommand(led),
                        new InstantCommand(),
                        () -> vision.autoCalibrate(follower, turret)
                )
        );
    }

    public Command intakeDuringPath(PathChain path) {
        return new ParallelDeadlineGroup(
                new AutoDriveCommand(follower, path),
                new IntakeCommand(intake, transit)
        ).andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));
    }

    public Command intakeDuringPath(PathChain path, double timeoutMs) {
        return new ParallelDeadlineGroup(
                new AutoDriveCommand(follower, path, timeoutMs),
                new IntakeCommand(intake, transit)
        ).andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));
    }

    @Override
    public void initialize() {
        this.follower = Constants.createFollower(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.transit = new Transit(hardwareMap, false);
        this.turret = new Turret(hardwareMap);
        this.vision = new Vision(hardwareMap);
        this.led = new Led(hardwareMap);
        this.alliance = Drive.Alliance.RED;

        follower.setStartingPose(new Pose(118.569, 128.559, Math.toRadians(38)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.569, 128.559),
                                new Pose(92.771, 98.674)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(90))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.771, 98.674),
                                new Pose(79.948, 57.252),
                                new Pose(123.628, 59.917)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.628, 59.917),
                                new Pose(86.404, 86.309)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(86.404, 86.309),
                                new Pose(104.313, 70.659),
                                new Pose(128.207, 64.201)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.207, 64.201),
                                new Pose(127.680, 55.921)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.680, 55.921),
                                new Pose(131.915, 54.896)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131.915, 54.896),
                                new Pose(86.792, 86.198)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(86.792, 86.198),
                                new Pose(103.648, 70.954),
                                new Pose(128.104, 64.100)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.104, 64.100),
                                new Pose(127.742, 55.877)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.742, 55.877),
                                new Pose(131.770, 54.850)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131.770, 54.850),
                                new Pose(86.825, 86.175)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(86.825, 86.175),
                                new Pose(124.093, 84.313)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.093, 84.313),
                                new Pose(86.707, 86.057)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(86.707, 86.057),
                                new Pose(65.128, 37.540),
                                new Pose(121.032, 35.950)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path15 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(121.032, 35.950),
                                new Pose(87.255, 88.380)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        schedule(
                new ParallelCommandGroup(
                        new TurretAlignCommand(follower, turret, alliance, vision),
                        new ShooterAlignCommand(follower, shooter, transit, alliance),
                        new SequentialCommandGroup(
                                new AutoDriveCommand(follower, Path1),
                                transitShootCommand(),
                                intakeDuringPath(Path2),
                                new AutoDriveCommand(follower, Path3),
                                transitShootCommand(),
                                new AutoDriveCommand(follower, Path4),
                                intakeDuringPath(Path5),
                                intakeDuringPath(Path6, 600),
                                new WaitCommand(500),
                                new AutoDriveCommand(follower, Path7),
                                transitShootCommand(),
                                new AutoDriveCommand(follower, Path8),
                                intakeDuringPath(Path9),
                                intakeDuringPath(Path10, 600),
                                new WaitCommand(500),
                                new AutoDriveCommand(follower, Path11),
                                transitShootCommand(),
                                intakeDuringPath(Path12),
                                new AutoDriveCommand(follower, Path13),
                                transitShootCommand(),
                                intakeDuringPath(Path14),
                                new AutoDriveCommand(follower, Path15),
                                transitShootCommand()
                        )
                )
        );
    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();
        telemetry.addData("Vision Pose: ", vision.getVisionPose());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Shooter at Setpoint: ", shooter.isShooterAtSetPoint());
        telemetry.update();
    }
}
