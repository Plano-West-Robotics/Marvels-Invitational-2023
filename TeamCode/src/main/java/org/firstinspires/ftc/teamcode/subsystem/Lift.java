package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.inchworm.PIDController;

public class Lift {
    private static boolean manual;

    final public static int POS_HIGH = -1860;
    final public static int POS_MID = -2000;
    final public static int POS_LOW = -1350;
    final public static int POS_DOWN = -6;
    final public static int MAX_HEIGHT = -1860;

    final public static int DEADZONE = 10;

    public final double Kp = 6;
    public final double Ki = 0;
    public final double Kd = 0.15;
    public final double F = 0;

    public static double MAX_VEL = 1478.9;
    public double power;

    public PIDController pid;
    public DcMotor leftSlide;
    private DcMotor rightSlide;
    private double target;

    private Telemetry telemetry;

    public Lift(Telemetry telemetry, HardwareMap hw) {
        pid = new PIDController(Kp, Ki, Kd, target);

        leftSlide = hw.get(DcMotor.class, "leftSlide");
        rightSlide = hw.get(DcMotor.class, "rightSlide");
        //rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
    }

    void setPower(double power) {
        power /= MAX_VEL;
        power = Range.clip(power, -1, 1) + F;
        this.power = power;
        double currentPos = getCurrentPos();

        if (Math.abs(target - currentPos) <= DEADZONE && Math.abs(currentPos) <= DEADZONE) {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
            return;
        }
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void setRawPower(double power) {
        if (leftSlide.getCurrentPosition() < MAX_HEIGHT && power < 0) return;
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void goTo(int target) {
        if (target > POS_DOWN) target = POS_DOWN;
        if (target < MAX_HEIGHT) target = MAX_HEIGHT;
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

    public int getCurrentPos() {
        return leftSlide.getCurrentPosition();
    }

    public void update(double stickVal) {
        int currentPos = leftSlide.getCurrentPosition();

        if (manual) {
            goTo((int)this.target + 2 * (int)(stickVal * 10));
            setPower(pid.calculate(currentPos));
        }
        else setPower(pid.calculate(currentPos));

        telemetry.addData("Slide Manual?", manual);
        telemetry.addData("Slide Power", power);
        telemetry.addData("Joystick", stickVal);
        telemetry.addData("Position", currentPos);
    }
}
