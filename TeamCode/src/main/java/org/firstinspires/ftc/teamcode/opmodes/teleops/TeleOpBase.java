package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.ShooterAlignCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

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
    public ElapsedTime timer;

    protected abstract Drive.Alliance getAlliance();

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap, getAlliance());
        gamepadEx1 = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);
        timer = new ElapsedTime();
        turret = new Turret(hardwareMap);
        timer.reset();

        new FunctionalButton(
                () -> timer.time(TimeUnit.SECONDS) == 90
        ).whenPressed(
                new InstantCommand(() -> gamepad1.rumble(1.0, 1.0, 800))
        );

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepadEx1));
        turret.setDefaultCommand(new TurretAlignCommand(drive, turret, getAlliance(),
                () -> gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)));
        shooter.setDefaultCommand(new ShooterAlignCommand(drive, shooter, getAlliance(),
                () -> gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)));

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.B)
        ).whenPressed(
                new InstantCommand(() -> drive.visionCalibrate())
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> drive.reset(0))
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5
        ).whenPressed(
                new InstantCommand(() -> intake.toggle())
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5
        ).whenHeld(
                new InstantCommand(() -> transit.setLimitServoState(Transit.LimitServoState.OPEN))
                        .alongWith(new InstantCommand(() -> drive.visionCalibrate()))
        ).whenReleased(
                new InstantCommand(() -> transit.setLimitServoState(Transit.LimitServoState.CLOSE))
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("VisionPose", drive.getVisionPose() != null);
        telemetry.addData("X", drive.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("Y",  drive.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("Heading", drive.getPose().getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset",drive.getYawOffset());
        telemetry.addData("ShooterVelocity", shooter.getShooterState().toString());
        telemetry.addData("Gamepad Lx: ", gamepadEx1.getLeftX());
        telemetry.addData("Gamepad Ly: ", gamepadEx1.getLeftY());
        telemetry.addData("Gamepad Rx: ", gamepadEx1.getRightX());
        telemetry.addData("LF Power: ", drive.leftBackMotor.getPower());
        telemetry.addData("RF Power: ", drive.rightFrontMotor.getPower());
        telemetry.addData("LB Power: ", drive.leftBackMotor.getPower());
        telemetry.addData("RB Motor: ", drive.rightBackMotor.getPower());
        telemetry.addData("LF Mode: ", drive.leftFrontMotor.getMode());
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ShooterVelocity", shooter.getVelocity());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
