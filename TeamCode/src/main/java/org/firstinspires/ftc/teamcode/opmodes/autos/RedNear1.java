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
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Autonomous(name = "Red Near 1")
public class RedNear1 extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Drive.Alliance alliance;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12;

    public Command shootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> vision.autoCalibrate(follower)),
                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new TransitCommand(shooter, transit)
                                .andThen(new WaitCommand(200))
                                .andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.FORWARD)))
                )
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

        follower.setStartingPose(new Pose(112.307, 135.917, Math.toRadians(270)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(112.307, 135.917),

                                new Pose(89.228, 87.523)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.228, 87.523),

                                new Pose(94.362, 60.320)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.362, 60.320),

                                new Pose(122.980, 59.384)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(122.980, 59.384),
                                new Pose(98.459, 65.338),
                                new Pose(127.802, 67.508)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127.802, 67.508),
                                new Pose(84.979, 52.841),
                                new Pose(88.623, 87.822)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.623, 87.822),

                                new Pose(93.118, 84.771)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(93.118, 84.771),

                                new Pose(120.016, 83.884)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.016, 83.884),

                                new Pose(88.798, 87.748)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.798, 87.748),

                                new Pose(96.238, 35.129)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.238, 35.129),

                                new Pose(120.452, 35.226)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.452, 35.226),

                                new Pose(88.532, 87.605)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.532, 87.605),

                                new Pose(105.653, 77.632)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        schedule(
                new ParallelCommandGroup(
                        new TurretAlignCommand(follower, turret, alliance),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                                new AutoDriveCommand(follower, Path1),
                                shootCommand(),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path12),
                                        new IntakeCommand(intake, transit)
                                ),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path2),
                                        new IntakeCommand(intake, transit)
                                ),
                                new ParallelRaceGroup(
                                    new AutoDriveCommand(follower, Path3),
                                    new IntakeCommand(intake, transit)
                                ),
                                new AutoDriveCommand(follower, Path4),
                                shootCommand(),
                                new AutoDriveCommand(follower, Path11),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path5),
                                        new IntakeCommand(intake, transit)
                                ),

                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path6),
                                        new IntakeCommand(intake, transit)
                                ),
                                shootCommand(),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path7),
                                        new IntakeCommand(intake, transit)
                                ),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path8),
                                        new IntakeCommand(intake, transit)
                                ),

                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path9),
                                        new IntakeCommand(intake, transit)
                                ),
                                shootCommand(),
                                new AutoDriveCommand(follower, Path10)
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
