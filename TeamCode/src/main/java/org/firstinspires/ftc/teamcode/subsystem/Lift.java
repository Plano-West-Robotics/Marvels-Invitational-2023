package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.inchworm.PIDController;

public class Lift {
    public static int currentPos;
    private static boolean manual;

    final public static int POS_HIGH = -3010;
    final public static int POS_MID = -2000;
    final public static int POS_LOW = -1350;
    final public static int POS_DOWN = -5;
    final public static int MAX_HEIGHT = 100;

    final public static int DEADZONE = 10;

    public final double Kp = 2;
    public final double Ki = 0.02;
    public final double Kd = 0.3;
    public final double F = 0.03;

    public static double MAX_VEL = 134.75;
    public double power;

    public PIDController pid;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private double target;

    private Telemetry telemetry;

    public Lift(Telemetry telemetry, HardwareMap hw) {
        pid = new PIDController(Kp, Ki, Kd, currentPos);
        pid.setTarget(currentPos);

        leftSlide = hw.get(DcMotor.class, "leftSlide");
        rightSlide = hw.get(DcMotor.class, "rightSlide");
        //rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
    }

    void setPower(double power) {
        power /= MAX_VEL;

        if (Math.abs(target - currentPos) <= DEADZONE && Math.abs(currentPos) <= DEADZONE) {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
            return;
        }
        leftSlide.setPower(Range.clip(power, -1, 1) + F);
        rightSlide.setPower(Range.clip(power, -1, 1) + F);
    }

    void setRawPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void goTo(int target) {
        this.target = target;
        pid.setTarget(target);
    }

    public void setManual(boolean manual) {
        Lift.manual = manual;
        pid.reset();
    }

    public void toggleManual() {
        setManual(!Lift.manual);
    }

    public void setParams(double Kp, double Ki, double Kd, double maxVel) {
        pid.setParams(Kp, Ki, Kd);
        MAX_VEL = maxVel;
    }

    public void update(float stickVal) {
        currentPos = leftSlide.getCurrentPosition();

        if (manual) setRawPower(stickVal);
        else setPower(pid.calculate(currentPos));



        telemetry.addData("Slide Manual?", manual);
        telemetry.addData("Slide Power", power);
        telemetry.addData("Joystick", stickVal);
        telemetry.addData("Position", currentPos);
    }
}
