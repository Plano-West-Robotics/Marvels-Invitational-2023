package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Arm {
    Servo leftServo;
    Servo rightServo;
    Servo wristServo;

    Telemetry telemetry;

    public static final List POS_PICKUP = new ArrayList<Double>() {
        {
            add(0d); // left servo
            add(0d); // right servo
            add(0d); // wrist servo
        }
    };

    public static final List POS_MID = new ArrayList<Double>() {
        {
            add(0d); // left servo
            add(0d); // right servo
            add(0d); // wrist servo
        }
    };

    public static final List POS_DROP = new ArrayList<Double>() {
        {
            add(0d); // left servo
            add(0d); // right servo
            add(0d); // wrist servo
        }
    };

    public Arm(Telemetry telemetry, HardwareMap hw) {
        leftServo = hw.get(Servo.class, "leftServo");
        rightServo = hw.get(Servo.class, "rightServo");
        wristServo = hw.get(Servo.class, "wristServo");
        this.telemetry = telemetry;
    }

    public void goTo(List pos) {
        try {
            leftServo.setPosition(pos.indexOf(0));
            rightServo.setPosition(pos.indexOf(1));
            wristServo.setPosition(pos.indexOf(2));
        } catch (Exception exception) {
            throw new RuntimeException("Please use the preset positions");
        }
    }
}
