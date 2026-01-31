package org.firstinspires.ftc.teamcode.opmodes.autos;

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
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Red Near 21")
public class RedNear21 extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Drive.Alliance alliance;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14;

    public Command transitShootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> vision.autoCalibrate(follower, turret)),
                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new TransitCommand(shooter, transit)
                                .andThen(new WaitCommand(200))
                                .andThen(new ShootCommand(intake, shooter))
                )
        );
    }

    public Command intakeCommand(PathChain path) {
        return new ParallelRaceGroup(
                new AutoDriveCommand(follower, path),
                new IntakeCommand(intake, transit)
        );
    }

    public Command cycleCommand() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, Path5),
                intakeCommand(Path6),
                intakeCommand(Path7),
                transitShootCommand()
        );
    }

    @Override
    public void initialize() {
        this.follower = Constants.createFollower(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.transit = new Transit(hardwareMap);
        this.turret = new Turret(hardwareMap);
        this.vision = new Vision(hardwareMap);
        this.alliance = Drive.Alliance.RED;

        follower.setStartingPose(new Pose(126.771, 119.048, Math.toRadians(36)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.771, 119.048),

                                new Pose(91.037, 90.612)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(91.037, 90.612),

                                new Pose(96.524, 59.963)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.524, 59.963),

                                new Pose(123.548, 59.405)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(123.548, 59.405),
                                new Pose(110.194, 66.496),
                                new Pose(123.300, 67.718)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(123.300, 67.718),
                                new Pose(93.292, 66.820),
                                new Pose(91.108, 90.678)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(91.108, 90.678),
                                new Pose(94.236, 81.912),
                                new Pose(125.440, 83.864)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.440, 83.864),

                                new Pose(91.214, 90.555)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(91.214, 90.555),

                                new Pose(92.818, 35.446)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(92.818, 35.446),

                                new Pose(122.948, 35.575)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(122.948, 35.575),

                                new Pose(90.947, 90.439)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.947, 90.439),

                                new Pose(113.806, 72.818)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        schedule(
                new ParallelCommandGroup(
                        new TurretAlignCommand(follower, turret, alliance, vision),
                        new ShooterAlignCommand(follower, shooter, alliance, () -> false),
                        new SequentialCommandGroup(
                                new AutoDriveCommand(follower, Path1),
                                transitShootCommand(),
                                intakeCommand(Path2),
                                intakeCommand(Path3),
                                intakeCommand(Path4),
                                intakeCommand(Path5),
                                transitShootCommand(),
                                intakeCommand(Path6),
                                intakeCommand(Path7),
                                transitShootCommand(),
                                intakeCommand(Path8),
                                intakeCommand(Path9),
                                intakeCommand(Path10),
                                transitShootCommand(),
                                new AutoDriveCommand(follower, Path11)
                        )
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Shooter at Setpoint: ", shooter.isShooterAtSetPoint());
        telemetry.update();
    }
}
