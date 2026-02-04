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
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    protected abstract Drive.Alliance getAlliance();
    public FtcDashboard dashboard;

    private void printPose() {
        TelemetryPacket packet = new TelemetryPacket();
        Pose2D pose = drive.getPose();

        double x = pose.getX(distanceUnit);
        double y = pose.getY(distanceUnit);
        double heading = pose.getHeading(angleUnit);

        double xDraw = 144 - x;
        double yDraw = 144 - y;

        double headingDraw = heading + Math.PI;

        headingDraw = Math.atan2(Math.sin(headingDraw), Math.cos(headingDraw));

        packet.put("x(raw)", x);
        packet.put("y(raw)", y);
        packet.put("heading(raw deg)", Math.toDegrees(heading));

        packet.put("x(draw)", xDraw);
        packet.put("y(draw)", yDraw);
        packet.put("heading(draw deg)", Math.toDegrees(headingDraw));

        packet.fieldOverlay().setFill("blue");
        packet.fieldOverlay().fillCircle(xDraw, yDraw, 2.0);

        double headingLength = 5.0;
        double hx = xDraw + Math.cos(headingDraw) * headingLength;
        double hy = yDraw + Math.sin(headingDraw) * headingLength;

        packet.fieldOverlay().setStroke("red");
        packet.fieldOverlay().strokeLine(xDraw, yDraw, hx, hy);

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap, getAlliance());
        gamepadEx1 = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);
        timer = new ElapsedTime();
        turret = new Turret(hardwareMap);
        vision = new Vision(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        timer.reset();

        new FunctionalButton(
                () -> timer.time(TimeUnit.SECONDS) == 90
        ).whenPressed(
                new InstantCommand(() -> gamepad1.rumble(1.0, 1.0, 800))
        );

        drive.setDefaultCommand(new DriveCommand(drive, gamepadEx1));
        turret.setDefaultCommand(new TurretAlignCommand(drive, turret, getAlliance(), vision, () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)));
        shooter.setDefaultCommand(new ShooterAlignCommand(drive, shooter, transit, getAlliance(), () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)));

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
                new TransitCommand(shooter, transit)
                        .andThen(new WaitCommand(200))
                        .andThen(new ShootCommand(intake, shooter))
        ).whenReleased(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.STOP)));

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenPressed(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenPressed(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST))
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
        CommandScheduler.getInstance().run();
        if (timer.milliseconds() - lastTime > 500) {
            aligning = vision.calibrate(drive, turret);
            lastTime = timer.milliseconds();
        }

        printPose();

        telemetry.addLine("----- Drive -----");
        telemetry.addData("Drive X: ", drive.getPose().getX(distanceUnit));
        telemetry.addData("Drive Y: ",  drive.getPose().getY(distanceUnit));
        telemetry.addData("Drive H: ", drive.getPose().getHeading(angleUnit));
        telemetry.addData("Drive Aligned: ", drive.getAligned());
        telemetry.addData("Aligning: ", aligning);

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
