package org.firstinspires.ftc.teamcode.JoshAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutonV1 extends OpMode {

    RobotMethods robot = new RobotMethods();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {

        robot.drive(0, 30, 0.5);
        robot.returnAfterBusy();
        if (robot.getDistance(1) <= 8) {
            robot.turn(90, 0.5);
            robot.returnAfterBusy();
            robot.drive(8, 0, 0.5);
            robot.returnAfterBusy();
            robot.moveLift(1000, 0.5, telemetry);
            robot.returnAfterBusy();
            robot.middle(1);
            sleep(1000);
            robot.middle(0);
        } else {
            robot.turn(90, 0.5);
            robot.returnAfterBusy();
            robot.drive(12, 0, 0.5);
            robot.returnAfterBusy();
        }

    }

    @Override
    public void loop() {

    }

    public void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            telemetry.addLine("Failed Sleep");
            telemetry.update();
        }
    }

}
