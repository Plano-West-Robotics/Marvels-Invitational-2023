package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    public static final double POS_CLOSE = 0;
    public static final double POS_OPEN = 0;

    Servo clawServo;
    Telemetry telemetry;

    public Claw(Telemetry telemetry, HardwareMap hw) {
        this.telemetry = telemetry;
        this.clawServo = hw.get(Servo.class, "clawServo");
    }

    public void goTo(double position) {
        clawServo.setPosition(position);
    }
}
