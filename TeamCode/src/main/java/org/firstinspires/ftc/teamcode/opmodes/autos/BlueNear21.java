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
        this.alliance = Drive.Alliance.BLUE;

        follower.setStartingPose(new Pose(17.229, 119.048, Math.toRadians(144)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.229, 119.048),

                                new Pose(52.963, 90.612)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.963, 90.612),

                                new Pose(47.476, 59.963)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(47.476, 59.963),

                                new Pose(20.452, 59.405)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.452, 59.405),
                                new Pose(33.806, 66.496),
                                new Pose(20.700, 67.718)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.700, 67.718),
                                new Pose(50.708, 66.820),
                                new Pose(52.892, 90.678)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.892, 90.678),
                                new Pose(49.764, 81.912),
                                new Pose(18.560, 83.864)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.560, 83.864),

                                new Pose(52.786, 90.555)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.786, 90.555),

                                new Pose(51.182, 35.446)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.182, 35.446),

                                new Pose(21.052, 35.575)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.052, 35.575),

                                new Pose(53.053, 90.439)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.053, 90.439),

                                new Pose(30.194, 72.818)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

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
