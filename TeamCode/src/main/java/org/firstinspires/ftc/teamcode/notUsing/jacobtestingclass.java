package org.firstinspires.ftc.teamcode.notUsing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutonMethods;

@Autonomous
public class jacobtestingclass extends LinearOpMode {

    @Override
    public void runOpMode() {
        DistanceSensor ir2m;
        ir2m = hardwareMap.get(DistanceSensor.class, "ir2m");

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("distance", ir2m.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }


    }
}
