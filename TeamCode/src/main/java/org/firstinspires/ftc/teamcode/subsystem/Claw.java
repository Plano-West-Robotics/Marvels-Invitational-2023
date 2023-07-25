package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    public static final double POS_OPEN = 0.3;
    public static final double POS_CLOSE = 1;

    private Servo clawServo;
    private double target;
    private Telemetry telemetry;

    public Claw(Telemetry telemetry, HardwareMap hw) {
        this.telemetry = telemetry;
        this.clawServo = hw.get(Servo.class, "clawServo");
        target = POS_CLOSE;
    }

    public void goTo(double position) {
        target = position;
        clawServo.setPosition(target);
        telemetry.addData("Claw position:", position);
    }

    public void toggle() {
        if (target == Claw.POS_CLOSE) {
            goTo(Claw.POS_OPEN);
        } else if (target == Claw.POS_OPEN) {
            goTo(Claw.POS_CLOSE);
        }
    }
}
