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

    public static final List<Double> POS_PICKUP = new ArrayList<Double>() {
        {
            add(1d); // left servo
            add(0d); // right servo
            add(0d); // wrist servo
        }
    };

    public static final List<Double> POS_DROP = new ArrayList<Double>() {
        {
            add(0d); // left servo
            add(1d); // right servo
            add(1d); // wrist servo
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
            leftServo.setPosition((double) pos.get(0));
            rightServo.setPosition((double) pos.get(1));
            wristServo.setPosition((double) pos.get(2));

            telemetry.addData("Left Servo: ", pos.get(0));
            telemetry.addData("Right Servo: ", pos.get(1));
            telemetry.addData("Wrist Servo: ", pos.get(2));
        } catch (Exception exception) {
            throw new RuntimeException("Please use the preset positions");
        }
    }
}
