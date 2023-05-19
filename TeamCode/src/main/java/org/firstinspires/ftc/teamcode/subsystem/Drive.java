package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Drive {
    public DcMotorEx fl, bl, br, fr;
    public BNO055IMU imu;
    private final VoltageSensor vsensor;
    @Nullable
    private final Telemetry tm;

    public Drive(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        fl = (DcMotorEx) hardwareMap.get("frontLeft");
        bl = (DcMotorEx) hardwareMap.get("backLeft");
        br = (DcMotorEx) hardwareMap.get("backRight");
        fr = (DcMotorEx) hardwareMap.get("frontRight");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motors = new DcMotorEx[]{fl, fr, bl, br};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        imu = (BNO055IMU) hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());
        yaw = imu.getAngularOrientation().thirdAngle;

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

    private double yaw;
    private double offset = 0;

    public double getYaw() {
        return yaw - offset;
    }

    public void resetYaw() {
        offset = getYaw();
    }

    private final double[] powers = new double[]{0, 0, 0, 0};

    public void setPowers(@NonNull double[] powers) {
        this.powers[0] = powers[0];
        this.powers[1] = powers[1];
        this.powers[2] = powers[2];
        this.powers[3] = powers[3];
    }

    @NonNull
    public double[] getPowers() {
        return powers.clone();
    }

    public void setVels(double x, double y, double rx) {
        final var rotX = x * cos(-getYaw()) - y * sin(-getYaw());
        final var rotY = x * sin(-getYaw()) + y * cos(-getYaw());

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        powers[0] = (rotY + rotX + rx) / denominator;
        powers[1] = (rotY - rotX + rx) / denominator;
        powers[2] = (rotY + rotX - rx) / denominator;
        powers[3] = (rotY - rotX - rx) / denominator;
    }
    public void setVelsNew(double x, double y, double rx, int i){
        final var rotX = x*cos(-getYaw()) - y*sin(-getYaw());
        final var rotY = x*sin(-getYaw()) + y*cos(-getYaw());
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        switch (i){
            case 0: {
                powers[i] = (rotY + rotX)/denominator;
            }
            break;
            case 1: {
                powers[i] = (rotY - rotX)/denominator;
            }
            break;
            case 2: {
                powers[i] = (rotY + rotX)/denominator;
            }
            break;
            case 3: {
                powers[i] = (rotY - rotX)/denominator;
            }
            break;
            default: {
                break;
            }
        }
    }

    public void update() {
        yaw = AngleUnit.normalizeRadians(imu.getAngularOrientation().firstAngle - offset);
        voltage = vsensor.getVoltage();

        final double velComp = 12 / getVoltage();

        fl.setPower(powers[0] * speed * velComp);
        bl.setPower(powers[1] * speed * velComp);
        br.setPower(powers[2] * speed * velComp);
        fr.setPower(powers[3] * speed * velComp);

        if (tm != null) {
            tm.addData("yaw", Math.toDegrees(getYaw()));
            tm.addData("firstAngle", Math.toDegrees(imu.getAngularOrientation().firstAngle));
            tm.addData("secondsAngle", Math.toDegrees(imu.getAngularOrientation().secondAngle));
            tm.addData("thirdAngle", Math.toDegrees(imu.getAngularOrientation().thirdAngle));
        }
    }
}
