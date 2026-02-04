package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kD;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kF;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kI;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.kP;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Util;

public class Shooter extends SubsystemBase {
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private Servo pitchServo;

    private ShooterState shooterState = ShooterState.STOP;
    private PitchState pitchState = PitchState.LOW;

    public Shooter(final HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        pitchServo = hardwareMap.get(Servo.class, ShooterConstants.pitchServoName);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        leftShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity),
        SLOW(ShooterConstants.slowVelocity),
        FAST(ShooterConstants.fastVelocity),
        DYNAMIC(0);

        double shooterVelocity;

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
        LOW(ShooterConstants.lowPose),
        NEAR1(ShooterConstants.near1Pose),
        NEAR2(ShooterConstants.near2Pose),
        NEAR3(ShooterConstants.near3Pose),
        NEAR4(ShooterConstants.near4Pose),
        NEAR5(ShooterConstants.near5Pose),
        FAR1(ShooterConstants.far1Pose),
        FAR2(ShooterConstants.far2Pose);

        final double servoPos;

        PitchState(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public void setPitchState(PitchState pitchState) {
        this.pitchState = pitchState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public void setDynamicSpeed(double dynamicSpeed) {
        setShooterState(ShooterState.DYNAMIC);
        shooterState.shooterVelocity =
                clip(dynamicSpeed, ShooterConstants.minVelocity, ShooterConstants.maxVelocity);
    }

    public double getVelocity() {
        return rightShooter.getVelocity();
    }

    public boolean isShooterAtSetPoint() {
        return Util.epsilonEqual(shooterState.shooterVelocity,
                -rightShooter.getVelocity(), ShooterConstants.shooterEpsilon);
    }

    public PitchState getPitchState() {
        return pitchState;
    }

    @Override
    public void periodic() {
//        if (shooterState == ShooterState.STOP) {
//            setPitchState(PitchState.LOW);
//        }
//        if (shooterState == ShooterState.SLOW) {
//            setPitchState(PitchState.MIDDLE);
//        }
//        if (shooterState == ShooterState.FAST) {
//            setPitchState(PitchState.HIGH);
//        }

        rightShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        leftShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        rightShooter.setVelocity(-shooterState.shooterVelocity);
        leftShooter.setVelocity(shooterState.shooterVelocity);
        pitchServo.setPosition(pitchState.servoPos);
    }
}
