package org.firstinspires.ftc.teamcode.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystem.Lift;

@Config
@Autonomous(group="tune")
public class LiftTuner extends LinearOpMode {
    public static double MAX_VEL = 1478.9;
    public static int HEIGHT = Lift.POS_MID;
    public static double Kp = 0;
    public static double Kd = 0;
    public static double Ki = 0;
    /*
     * This class should be used to tune turn PID for InchWorm.
     * Requires a gamepad. Make sure to write down the tuned values, or they will be lost forever.
     */
    @Override
    public void runOpMode() {

        Lift lift = new Lift(telemetry, hardwareMap);
        lift.setManual(false);
        lift.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Kp = lift.Kp;
        Ki = lift.Ki;
        Kd = lift.Kd;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();
        while (opModeIsActive()) {
            int current = lift.getCurrentPos();
            lift.setParams(Kp, Ki, Kd, MAX_VEL);

            lift.goTo(HEIGHT);
            lift.update(-1);

            telemetry.addData("target", HEIGHT);
            //telemetry.addData("out", out);
            telemetry.addData("error", String.format("%.2f", (double)HEIGHT - (double)current));
            telemetry.addData("current", String.format("%.2f", (double)current));
            telemetry.addData("Kp", String.format("%.2f", Kp));
            telemetry.addData("Ki", String.format("%.2f", Ki));
            telemetry.addData("Kd", String.format("%.2f", Kd));
            telemetry.addData("Power", String.format("%.2f", lift.power));
            telemetry.update();
        }
    }
}
