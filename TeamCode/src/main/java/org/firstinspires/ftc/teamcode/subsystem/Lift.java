package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.inchworm.PIDController;

public class Lift {
    public static int currentPos;
    private static boolean manual;

    final public static int POS_HIGH = 0;
    final public static int POS_MID = 0;
    final public static int POS_LOW = 0;
    final public static int POS_DOWN = 0;
    final public static int MAX_HEIGHT = 0;

    public PIDController pid;
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private Telemetry telemetry;

    public Lift(Telemetry telemetry, HardwareMap hw) {
        pid = new PIDController(0, 0, 0, currentPos);
        pid.setTarget(currentPos);

        leftSlide = hw.get(DcMotor.class, "leftSlide");
        rightSlide = hw.get(DcMotor.class, "rightSlide");

        this.telemetry = telemetry;
    }

    void setPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void goTo(int target) {
        pid.setTarget(target);
        pid.reset();
    }

    public void setManual(boolean manual) {
        Lift.manual = manual;
        pid.reset();
    }

    public void toggleManual() {
        setManual(!Lift.manual);
    }

    public void update(double stickVal) {
        if (Lift.manual) {
            setPower(stickVal);
        } else {
            setPower(pid.calculate(currentPos));
        }

        currentPos = leftSlide.getCurrentPosition();
    }
}
