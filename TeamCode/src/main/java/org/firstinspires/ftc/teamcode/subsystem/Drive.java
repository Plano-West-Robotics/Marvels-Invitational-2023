package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.inchworm.InchWorm;


public class Drive {
    public DcMotorEx fl, bl, br, fr;
    private final IMU imu;
    private final VoltageSensor vsensor;
    @Nullable
    private final Telemetry tm;

    public Drive(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motors = new DcMotorEx[]{fl, fr, bl, br};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(InchWorm.GLOBAL_ORIENTATION));

        vsensor = hardwareMap.voltageSensor.iterator().next();
        voltage = vsensor.getVoltage();

        tm = telemetry;
    }

    public Drive(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    private double speed = 1;

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }

    private double voltage;

    public double getVoltage() {
        return voltage;
    }

    private double offset = 0;

    /**
     * Returns the current yaw
     * @return Current yaw of the robot
     */
    public double getYaw() {
        return getYaw(AngleUnit.RADIANS);
    }

    /**
     * Returns the current yaw in the specified unit
     * @param angleUnit Unit
     * @return current yaw of the robot in the specified unit
     */
    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit) - offset;
    }

    public void resetYaw() {
        offset = getYaw();
    }

    /**
     * Sets the velocity of the wheels
     * @param x Strafe power
     * @param y Forward power
     * @param turn Turn power
     * @param volComp Voltage compensation
     */
    public void setVels(double x, double y, double turn, double volComp) {
        double rotX = x * cos(getYaw()) - y * sin(getYaw());
        double rotY = x * sin(getYaw()) + y * cos(getYaw());

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        fl.setPower((rotY + rotX + turn) / denominator * speed * volComp);
        bl.setPower((rotY - rotX + turn) / denominator * speed * volComp);
        br.setPower((rotY + rotX - turn) / denominator * speed * volComp);
        fr.setPower((rotY - rotX - turn) / denominator * speed * volComp);
    }

    /**
     * Update drive velocities
     * @param x Strafe power
     * @param y Forward power
     * @param turn Turning power
     */
    public void update(double x, double y, double turn) {
        voltage = vsensor.getVoltage();

        double volComp = 12 / getVoltage();

        setVels(x, y, turn, volComp);

        if (tm != null) {
            tm.addData("yaw", getYaw(AngleUnit.DEGREES));
        }
    }
}