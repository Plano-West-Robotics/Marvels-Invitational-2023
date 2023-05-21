package org.firstinspires.ftc.teamcode.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Lift;

@Config
@Autonomous(group="tune")
public class LiftTuner extends LinearOpMode {
    public static final double MAX_ANG_VEL = -188;
    public static int HEIGHT = 10;
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

        Kp = lift.Kp;
        Ki = lift.Ki;
        Kd = lift.Kd;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();
        while (opModeIsActive()) {
            //double current = inchWorm.tracker.currentPos.theta.angleInDegrees();
            int current = Lift.currentPos;
            lift.pid.setParams(Kp, Ki, Kd);
            /*if (gamepad1.x) {
                out = 0;
                controller.reset();
            }*/

            lift.goTo(HEIGHT);

            telemetry.addData("target", HEIGHT);
            //telemetry.addData("out", out);
            telemetry.addData("error", String.format("%.2f", (double)HEIGHT - (double)current));
            telemetry.addData("current", String.format("%.2f", (double)current));
            telemetry.addData("Kp", String.format("%.2f", Kp));
            telemetry.addData("Ki", String.format("%.2f", Ki));
            telemetry.addData("Kd", String.format("%.2f", Kd));
            telemetry.addData("Power", String.format("%.2f", lift.power));
            telemetry.update();

            lift.update(gamepad1.left_stick_y);
        }
    }
}
