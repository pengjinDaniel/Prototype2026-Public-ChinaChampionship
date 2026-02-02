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

@Autonomous(name = "Blue Near 21")
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
                        new WaitCommand(1850),
                        new TransitCommand(shooter, transit)
                                .andThen(new WaitCommand(100))
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
        this.alliance = Drive.Alliance.BLUE;

        follower.setStartingPose(new Pose(31.905, 134.641, Math.toRadians(90)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(31.905, 134.641),

                                new Pose(51.474, 91.462)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.474, 91.462),

                                new Pose(47.547, 59.787)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(47.547, 59.787),

                                new Pose(18.505, 59.557)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.505, 59.557),
                                new Pose(41.047, 66.245),
                                new Pose(51.383, 91.524)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.383, 91.524),
                                new Pose(45.944, 63.811),
                                new Pose(20.610, 64.236)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.610, 64.236),
                                new Pose(29.586, 55.749),
                                new Pose(14.038, 51.687)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(14.038, 51.687),
                                new Pose(40.801, 64.085),
                                new Pose(51.697, 91.798)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.697, 91.798),

                                new Pose(45.332, 83.759)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.332, 83.759),

                                new Pose(20.786, 83.554)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.786, 83.554),

                                new Pose(51.644, 91.502)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.644, 91.502),

                                new Pose(50.910, 35.369)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.910, 35.369),

                                new Pose(18.149, 35.777)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.149, 35.777),

                                new Pose(51.679, 91.718)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.679, 91.718),

                                new Pose(31.923, 71.968)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
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
                                transitShootCommand(),
                                intakeCommand(Path5),
                                intakeCommand(Path6),
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
        CommandScheduler.getInstance().run();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Shooter at Setpoint: ", shooter.isShooterAtSetPoint());
        telemetry.update();
    }
}
