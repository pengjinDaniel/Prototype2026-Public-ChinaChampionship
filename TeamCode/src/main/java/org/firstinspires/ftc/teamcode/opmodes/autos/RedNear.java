package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
                new TransitCommand(shooter, transit, intake),
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

        follower.setStartingPose(new Pose(118.169, 128.559, Math.toRadians(38)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.169, 128.559),
                                new Pose(93.770, 119.245)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(90))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(93.770, 119.245),
                                new Pose(63.171, 63.244),
                                new Pose(123.628, 59.917)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.628, 59.917),
                                new Pose(92.795, 81.516)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.795, 81.516),
                                new Pose(101.517, 68.662),
                                new Pose(128.606, 65.999)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.606, 65.999),
                                new Pose(124.085, 63.710)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .setTimeoutConstraint(100)
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.085, 63.710),
                                new Pose(131.716, 57.293)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .setTimeoutConstraint(100)
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(131.716, 57.293),
                                new Pose(91.473, 65.051),
                                new Pose(92.584, 81.205)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.584, 81.205),
                                new Pose(101.850, 68.557),
                                new Pose(128.503, 65.897)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.503, 65.897),
                                new Pose(123.947, 64.065)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .setTimeoutConstraint(100)
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.947, 64.065),
                                new Pose(131.570, 57.447)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .setTimeoutConstraint(100)
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(131.570, 57.447),
                                new Pose(91.321, 65.915),
                                new Pose(92.218, 83.778)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(92.218, 83.778),
                                new Pose(124.492, 83.714)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.492, 83.714),
                                new Pose(92.100, 83.860)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(92.100, 83.860),
                                new Pose(65.128, 37.540),
                                new Pose(121.032, 35.950)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path15 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(121.032, 35.950),
                                new Pose(101.659, 92.871)
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
                                intakeDuringPath(Path6),
                                new AutoDriveCommand(follower, Path7),
                                transitShootCommand(),
                                new AutoDriveCommand(follower, Path8),
                                intakeDuringPath(Path9),
                                intakeDuringPath(Path10),
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
