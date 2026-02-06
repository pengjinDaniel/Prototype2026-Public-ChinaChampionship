package org.firstinspires.ftc.teamcode.opmodes.teleops;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.angleUnit;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.distanceUnit;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.nearGoalDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterAlignCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.concurrent.TimeUnit;

@Config
@Configurable
public abstract class TeleOpBase extends CommandOpMode {
    public Drive drive;
    public GamepadEx gamepadEx1;
    public Shooter shooter;
    public Transit transit;
    public Intake intake;
    public Turret turret;
    public Vision vision;
    public ElapsedTime timer;
    public boolean aligning = false;
    public double lastTime = 0;
    public boolean killing = false;
    public boolean killed = false;

    protected abstract Drive.Alliance getAlliance();

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap, getAlliance());
        gamepadEx1 = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap, true);
        intake = new Intake(hardwareMap);
        timer = new ElapsedTime();
        turret = new Turret(hardwareMap);
        vision = new Vision(hardwareMap);
        timer.reset();

        new FunctionalButton(
                () -> timer.time(TimeUnit.SECONDS) == 90
        ).whenPressed(
                new InstantCommand(() -> gamepad1.rumble(1.0, 1.0, 800))
        );

        drive.setDefaultCommand(new DriveCommand(drive, gamepadEx1));
        turret.setDefaultCommand(new TurretAlignCommand(drive, turret, getAlliance(), vision, () -> killed));
        shooter.setDefaultCommand(new ShooterAlignCommand(drive, shooter, transit, getAlliance(), () -> killed));

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> drive.reset(0))
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5
        ).whenHeld(
                new IntakeCommand(intake, transit)
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5
        ).whenHeld(
                new ParallelCommandGroup(
                        new InstantCommand(() -> drive.setScoring(true)),
                        new TransitCommand(shooter, transit, intake)
                )
        ).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> drive.setScoring(false)),
                        new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP))
                )
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenPressed(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW))
                        .alongWith(new InstantCommand(() -> shooter.setShooterLimitState(Shooter.ShooterLimitState.NEAR)))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenPressed(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST))
                        .alongWith(new InstantCommand(() -> shooter.setShooterLimitState(Shooter.ShooterLimitState.FAR)))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whenPressed(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSED))
        ).whenReleased(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)
        ).whenPressed(
                new InstantCommand(() -> turret.modify(2000))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
        ).whenPressed(
                new InstantCommand(() -> turret.modify(-2000))
        );
    }

    @Override
    public void run() {
        killing = gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) || killing;
        CommandScheduler.getInstance().run();
        if (timer.milliseconds() - lastTime > 500 && !killing) {
            aligning = vision.calibrate(drive, turret);
            lastTime = timer.milliseconds();
        }

        telemetry.addLine("----- Drive -----");
        telemetry.addData("Drive X: ", drive.getPose().getX(distanceUnit));
        telemetry.addData("Drive Y: ",  drive.getPose().getY(distanceUnit));
        telemetry.addData("Drive H: ", drive.getPose().getHeading(angleUnit));
        telemetry.addData("Drive Aligned: ", drive.getAligned());
        telemetry.addData("Aligning: ", aligning);
        telemetry.addData("X Velocity", drive.getFilteredVelX());
        telemetry.addData("Y Velocity", drive.getFilteredVelY());
        telemetry.addData("H Velocity", drive.getFilteredHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

        telemetry.addLine("----- Vision -----");
        telemetry.addData("Vision X: ", vision.getVisionPose().getX(distanceUnit));
        telemetry.addData("Vision Y: ", vision.getVisionPose().getY(distanceUnit));
        telemetry.addData("Vision H: ", vision.getVisionPose().getHeading(angleUnit));

        telemetry.addLine("----- Shooter -----");
        telemetry.addData("Shooter State: ", shooter.getShooterState());
        telemetry.addData("Shooter Target: ", shooter.getShooterState().getShooterVelocity());
        telemetry.addData("Shooter Speed: ", shooter.getVelocity());
        telemetry.addData("Shooter at SetPoint: ", shooter.isShooterAtSetPoint());

        telemetry.addLine("----- Turret -----");
        telemetry.addData("Encoder Pos: ", turret.getTurretPos());
        telemetry.addData("Turret SetPoint: ", turret.getTicksSetpoint());
        telemetry.addData("Distance to Goal: ", Util.goalInTurretSys(drive.getPose(), drive.getAlliance()).getMagnitude());

        telemetry.update();
    }
}
