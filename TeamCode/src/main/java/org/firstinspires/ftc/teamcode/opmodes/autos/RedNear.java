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

@Autonomous(name = "Red Near")
public class RedNear extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Drive.Alliance alliance;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12;

    @Override
    public void initialize() {
        this.follower = Constants.createFollower(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.transit = new Transit(hardwareMap);
        this.turret = new Turret(hardwareMap);
        this.vision = new Vision(hardwareMap);
        this.alliance = Drive.Alliance.RED;

        follower.setStartingPose(new Pose(115.982, 131.581, Math.toRadians(36)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(115.982, 131.581),

                                new Pose(94.712, 93.004)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.712, 93.004),

                                new Pose(95.710, 59.694)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.710, 59.694),

                                new Pose(121.075, 59.353)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(121.075, 59.353),
                                new Pose(98.520, 66.831),
                                new Pose(126.886, 67.162)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(126.886, 67.162),
                                new Pose(73.808, 62.109),
                                new Pose(94.808, 92.668)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.808, 92.668),

                                new Pose(94.716, 83.230)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.716, 83.230),

                                new Pose(121.013, 83.824)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(121.013, 83.824),

                                new Pose(94.750, 93.127)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.750, 93.127),

                                new Pose(95.703, 32.724)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.703, 32.724),

                                new Pose(125.239, 32.845)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.239, 32.845),

                                new Pose(95.406, 92.802)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.406, 92.802),

                                new Pose(109.204, 72.391)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        schedule(
                new ParallelCommandGroup(
                        new TurretAlignCommand(follower, turret, alliance),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                                new AutoDriveCommand(follower, Path1),
                                new InstantCommand(() -> vision.autoCalibrate(follower)),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2500),
                                        new TransitCommand(shooter, transit)
                                                .andThen(new WaitCommand(200))
                                                .andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.FORWARD)))
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
                                new AutoDriveCommand(follower, Path5),
                                new InstantCommand(() -> vision.autoCalibrate(follower)),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2500),
                                        new TransitCommand(shooter, transit)
                                                .andThen(new WaitCommand(200))
                                                .andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.FORWARD)))
                                        ),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path6),
                                        new IntakeCommand(intake, transit)
                                ),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path7),
                                        new IntakeCommand(intake, transit)
                                ),
                                new AutoDriveCommand(follower, Path8),
                                new InstantCommand(() -> vision.autoCalibrate(follower)),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2500),
                                        new TransitCommand(shooter, transit)
                                                .andThen(new WaitCommand(200))
                                                .andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.FORWARD)))
                                        ),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path9),
                                        new IntakeCommand(intake, transit)
                                ),
                                new ParallelRaceGroup(
                                        new AutoDriveCommand(follower, Path10),
                                        new IntakeCommand(intake, transit)
                                ),
                                new AutoDriveCommand(follower, Path11),
                                new InstantCommand(() -> vision.autoCalibrate(follower)),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(2500),
                                        new TransitCommand(shooter, transit)
                                                .andThen(new WaitCommand(200))
                                                .andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.FORWARD)))
                                        ),
                                new AutoDriveCommand(follower, Path12)
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
