package org.firstinspires.ftc.teamcode.tune;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(group="tune")
public class LiftSpeedTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        double veloSum = 0;
        int numMeasurements = 0;

        double time = getRuntime() + 1;
        leftSlide.setPower(-1);
        while (getRuntime() < time) {
            double velo = leftSlide.getVelocity();
            veloSum += velo;
            numMeasurements++;
        }

        leftSlide.setPower(0);
        while (opModeIsActive()) {
            telemetry.addData("lift max velocity", veloSum / numMeasurements);
            telemetry.update();
        }
    }
}
