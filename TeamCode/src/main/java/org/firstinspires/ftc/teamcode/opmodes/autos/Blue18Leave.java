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

@Autonomous(name = "Blue 18 Leave", group = "Auto")
public class Blue18Leave extends CommandOpMode {
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
        this.alliance = Drive.Alliance.BLUE;

        follower.setStartingPose(new Pose(31.493, 135.917, Math.toRadians(270)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(31.493, 135.917),

                                new Pose(45.944, 116.774)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(45.944, 116.774),
                                new Pose(89.801, 58.463),
                                new Pose(22.155, 59.180)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(22.155, 59.180),
                                new Pose(45.977, 57.388),
                                new Pose(55.516, 87.846)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.516, 87.846),
                                new Pose(48.695, 67.298),
                                new Pose(24.000, 66.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.000, 66.000),

                                new Pose(16.408, 57.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(16.408, 57.500),
                                new Pose(47.815, 62.972),
                                new Pose(55.560, 87.725)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.560, 87.725),
                                new Pose(44.400, 83.866),
                                new Pose(22.563, 84.284)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.563, 84.284),

                                new Pose(55.579, 87.638)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.579, 87.638),
                                new Pose(57.820, 70.602),
                                new Pose(56.921, 42.241),
                                new Pose(66.846, 36.409),
                                new Pose(22.779, 35.313)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.779, 35.313),

                                new Pose(58.870, 117.499)
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
