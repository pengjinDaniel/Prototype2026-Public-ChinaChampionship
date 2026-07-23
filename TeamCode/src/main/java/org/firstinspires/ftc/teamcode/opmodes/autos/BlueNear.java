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

@Autonomous(name = "Blue Near", group = "Auto")
public class BlueNear extends CommandOpMode {
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
        this.alliance = Drive.Alliance.BLUE;

        follower.setStartingPose(new Pose(25.431, 128.559, Math.toRadians(142)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.431, 128.559),
                                new Pose(51.229, 98.674)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(-90))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.229, 98.674),
                                new Pose(64.052, 57.252),
                                new Pose(20.372, 59.917)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.372, 59.917),
                                new Pose(57.596, 86.309)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(57.596, 86.309),
                                new Pose(39.687, 70.659),
                                new Pose(15.793, 64.201)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.793, 64.201),
                                new Pose(16.320, 55.921)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.320, 55.921),
                                new Pose(12.085, 54.896)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.085, 54.896),
                                new Pose(57.208, 86.198)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(57.208, 86.198),
                                new Pose(40.352, 70.954),
                                new Pose(15.896, 64.100)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.896, 64.100),
                                new Pose(16.258, 55.877)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.258, 55.877),
                                new Pose(12.230, 54.850)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.230, 54.850),
                                new Pose(57.175, 86.175)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.175, 86.175),
                                new Pose(19.907, 84.313)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.907, 84.313),
                                new Pose(57.293, 86.057)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(57.293, 86.057),
                                new Pose(78.872, 37.540),
                                new Pose(22.968, 35.950)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        Path15 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.968, 35.950),
                                new Pose(56.745, 88.380)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
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
