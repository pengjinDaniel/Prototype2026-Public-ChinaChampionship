package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kD;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kF;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kI;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Util;

public class Shooter extends SubsystemBase {
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private Servo pitchServo;

    private ShooterState shooterState = ShooterState.STOP;

    public double dynamicSpeed;

    public Shooter(final HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        pitchServo = hardwareMap.get(Servo.class, ShooterConstants.pitchServoName);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        leftShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        pitchServo.setPosition(PitchState.LOW.servoPos);
    }

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity),
        SLOW(ShooterConstants.slowVelocity),
        FAST(ShooterConstants.fastVelocity),
        DYNAMIC(0);

        final double shooterVelocity;

        public double getShooterVelocity() {
            return shooterVelocity;
        }

        ShooterState(double shooterVelocity) {
            this.shooterVelocity = shooterVelocity;
        }
    }

    public enum PitchState {
        HIGH(ShooterConstants.highPose),
        MIDDLE(ShooterConstants.middlePose),
        LOW(ShooterConstants.lowPose);

        final double servoPos;

        PitchState(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public void setPitchState(PitchState pitchState) {
        pitchServo.setPosition(pitchState.servoPos);
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public void setDynamicSpeed(double dynamicSpeed) {
        this.dynamicSpeed = dynamicSpeed;
    }

    public double getVelocity() {
        return rightShooter.getVelocity();
    }

    public boolean isShooterAtSetPoint() {
        return Util.epsilonEqual(shooterState.shooterVelocity,
                -rightShooter.getVelocity(), ShooterConstants.shooterEpsilon);
    }

    @Override
    public void periodic() {
        if (shooterState != ShooterState.DYNAMIC) {
            rightShooter.setVelocity(-shooterState.shooterVelocity);
            leftShooter.setVelocity(shooterState.shooterVelocity);
            if (shooterState == ShooterState.STOP) {
                setPitchState(PitchState.LOW);
            }
            if (shooterState == ShooterState.SLOW) {
                setPitchState(PitchState.MIDDLE);
            }
            if (shooterState == ShooterState.FAST) {
                setPitchState(PitchState.HIGH);
            }
        }
        else {
            rightShooter.setVelocity(dynamicSpeed);
            leftShooter.setVelocity(dynamicSpeed);
        }
    }
}
