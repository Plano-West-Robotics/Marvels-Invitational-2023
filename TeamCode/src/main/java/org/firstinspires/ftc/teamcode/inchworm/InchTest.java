package org.firstinspires.ftc.teamcode.inchworm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.inchworm.units.Angle;
import org.firstinspires.ftc.teamcode.inchworm.units.Distance;

@Autonomous(group = "test")
public class InchTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        WormUtil wormUtil = new WormUtil(this, InchWorm.GLOBAL_ORIENTATION);
        InchWorm inchWorm = new InchWorm(this,
                InchWorm.GLOBAL_ORIENTATION,
                InchWorm.POSE_ZERO);

        wormUtil.waitForStart();
        inchWorm.moveTo(new InchWorm.Pose(Distance.ticks(1000), Distance.ZERO, Angle.degrees(270)));
        inchWorm.moveTo(new InchWorm.Pose(Distance.ZERO, Distance.ZERO));
        inchWorm.moveTo(new InchWorm.Pose(Distance.ZERO, Distance.ticks(1000), Angle.degrees(90)));
        inchWorm.moveTo(new InchWorm.Pose(Distance.ZERO, Distance.ZERO));

        inchWorm.moveTo(new InchWorm.Pose(Distance.ZERO, Distance.ZERO, Angle.ZERO));
    }
}
