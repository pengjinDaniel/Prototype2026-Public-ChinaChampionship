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

@Autonomous(name = "Blue Near 21", group = "Auto")
public class BlueNear21 extends CommandOpMode {
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
                        new WaitCommand(1900),
                        new TransitCommand(shooter, transit, intake)
                                .andThen(new WaitCommand(150))
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
        this.transit = new Transit(hardwareMap);
        this.turret = new Turret(hardwareMap);
        this.vision = new Vision(hardwareMap);
        this.alliance = Drive.Alliance.BLUE;

        follower.setStartingPose(new Pose(31.693, 134.641, Math.toRadians(90)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(31.693, 134.641),

                                new Pose(33.394, 111.456)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(33.394, 111.456),
                                new Pose(49.596, 110.360),
                                new Pose(48.610, 61.276)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.610, 61.276),

                                new Pose(17.654, 60.833)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(17.654, 60.833),
                                new Pose(41.047, 66.245),
                                new Pose(51.170, 91.312)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.170, 91.312),
                                new Pose(47.433, 65.300),
                                new Pose(21.035, 65.938)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(21.035, 65.938),
                                new Pose(34.479, 62.555),
                                new Pose(18.080, 53.814)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.080, 53.814),
                                new Pose(40.801, 64.085),
                                new Pose(51.059, 91.585)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.059, 91.585),

                                new Pose(45.758, 83.972)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.758, 83.972),

                                new Pose(20.786, 83.767)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.786, 83.767),

                                new Pose(51.431, 91.502)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.431, 91.502),

                                new Pose(50.910, 36.220)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.910, 36.220),

                                new Pose(18.149, 35.990)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.149, 35.990),

                                new Pose(50.829, 91.505)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.829, 91.505),

                                new Pose(31.923, 71.968)
                        )
                ).setTangentHeadingInterpolation()

                .build();


        schedule(
                new ParallelCommandGroup(
                        new TurretAlignCommand(follower, turret, alliance, vision),
                        new ShooterAlignCommand(follower, shooter, transit, alliance),
                        new SequentialCommandGroup(
                                new AutoDriveCommand(follower, Path1)
                                        .alongWith(transitShootCommand()),
                                intakeCommand(Path2),
                                intakeCommand(Path3),
                                intakeCommand(Path4),
                                transitShootCommand(),
                                intakeTimedCommand(Path5, 2000),
                                intakeTimedCommand(Path6, 1000),
                                new ParallelDeadlineGroup(
                                        new WaitCommand(500),
                                        new IntakeCommand(intake, transit)
                                ),
                                intakeCommand(Path7),
                                transitShootCommand(),
                                intakeCommand(Path8),
                                intakeCommand(Path9),
                                intakeCommand(Path10),
                                transitShootCommand(),
                                intakeCommand(Path11),
                                intakeCommand(Path12),
                                intakeCommand(Path13),
                                transitShootCommand(),
                                intakeCommand(Path14)
                        )
                )
        );
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
    }
}
