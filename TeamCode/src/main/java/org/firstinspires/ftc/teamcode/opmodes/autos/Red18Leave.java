package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import com.pedropathing.paths.Path;
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

@Autonomous(name = "Red 18 Leave", group = "Auto")
public class Red18Leave extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private Turret turret;
    private Vision vision;
    private Led led;
    private Drive.Alliance alliance;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14;

    public Command transitShootCommand() {
        return new SequentialCommandGroup(
                new TransitCommand(shooter, transit, intake),
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)),
                new ConditionalCommand(new LedWinkCommand(led), new InstantCommand(), () -> vision.autoCalibrate(follower, turret))
        );
    }

    public Command intakeTrajCommand(PathChain path) {
        return new ParallelRaceGroup(
                new AutoDriveCommand(follower, path),
                new IntakeCommand(intake, transit)
        ).andThen(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));
    }

    public Command intakeTimedCommand(PathChain path, double time) {
        return new ParallelRaceGroup(
                new AutoDriveCommand(follower, path, time),
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

        follower.setStartingPose(new Pose(112.108, 135.917, Math.toRadians(-90)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(112.108, 135.917),

                                new Pose(98.056, 116.774)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(98.056, 116.774),
                                new Pose(70.202, 58.019),
                                new Pose(121.545, 58.181)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(121.545, 58.181),
                                new Pose(94.727, 57.588),
                                new Pose(88.484, 87.846)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.484, 87.846),
                                new Pose(95.505, 65.900),
                                new Pose(125.000, 64.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.000, 64.000),

                                new Pose(132.000, 54.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 54.000),
                                new Pose(95.886, 62.473),
                                new Pose(88.440, 87.725)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.440, 87.725),
                                new Pose(100.599, 82.767),
                                new Pose(122.236, 83.585)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(122.236, 83.585),

                                new Pose(88.421, 87.638)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.421, 87.638),
                                new Pose(86.180, 70.602),
                                new Pose(87.079, 42.241),
                                new Pose(77.154, 36.409),
                                new Pose(121.221, 35.313)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(121.221, 35.313),

                                new Pose(93.810, 123.767)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();


        schedule(
                new ParallelCommandGroup(
                        new TurretAlignCommand(follower, turret, alliance, vision),
                        new ShooterAlignCommand(follower, shooter, transit, alliance),
                        new SequentialCommandGroup(
                                new AutoDriveCommand(follower, Path1),
                                transitShootCommand(),
                                intakeTrajCommand(Path2),
                                intakeTrajCommand(Path3),
                                transitShootCommand(),
                                intakeTimedCommand(Path4, 2000),
                                intakeTimedCommand(Path5, 1000),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(500),
                                        new IntakeCommand(intake, transit)
                                ),
                                intakeTrajCommand(Path6),
                                transitShootCommand(),
                                intakeTimedCommand(Path4, 2000),
                                intakeTimedCommand(Path5, 1000),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(500),
                                        new IntakeCommand(intake, transit)
                                ),
                                intakeTrajCommand(Path6),
                                transitShootCommand(),
                                intakeTrajCommand(Path7),
                                intakeTrajCommand(Path8),
                                transitShootCommand(),
                                intakeTrajCommand(Path9),
                                intakeTrajCommand(Path10),
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
