package org.firstinspires.ftc.teamcode.OpenHouse;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class OpenHousePuzzle2 extends LinearOpMode {

    private AutonMethods robot = new AutonMethods();

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. The init() method of the hardware class does all the work here
        robot.init(hardwareMap, telemetry, false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
        if (gamepad1.a) {
            robot.drive(2*robot.feet, 0,.25);
            robot.drive(2*robot.feet, 0,.25);
            robot.drive(2*robot.feet, 0,.25);
            robot.drive(0, 2*robot.feet,.25);
            robot.drive(-2*robot.feet, 0,.25);
        }
        else if (gamepad1.b) {
            robot.drive(0, -2*robot.feet,.25);
            sleep(2000);
            robot.drive(4*robot.feet, 0,.25);
        }
        else if (gamepad1.x) {
            robot.drive(0, 2*robot.feet,.25);
            robot.drive(2*robot.feet, 0,.25);
            robot.turn(-180,.25);
        }
        else if (gamepad1.y) {
            robot.turn(90, .25);
            robot.drive(4*robot.feet, 0,.25);
            robot.drive(-2*robot.feet, 0,.25);
            robot.turn(-90,.25);
        }
        else {
            robot.teleOpDrive(gamepad1);
        }

            // To prevent button mashing and multiple executions

            // Include telemetry updates or additional controls if needed
            telemetry.update();
        }
    }
}
