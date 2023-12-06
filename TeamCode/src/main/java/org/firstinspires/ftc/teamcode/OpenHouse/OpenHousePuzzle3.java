package org.firstinspires.ftc.teamcode.OpenHouse;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutonMethods;

@TeleOp
public class OpenHousePuzzle3 extends LinearOpMode {

    private AutonMethods robot = new AutonMethods();
    private String currentMode = ""; // This will keep track of the current sequence
    private int step = 0; // This will keep track of the steps within a sequence

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, false);
        waitForStart();

        while (opModeIsActive()) {
            // Check if a sequence is currently running or if a button is pressed to start a sequence
            if (currentMode.equals("") && gamepad1.a) {
                currentMode = "A";
            } else if (currentMode.equals("") && gamepad1.b) {
                currentMode = "B";
            } else if (currentMode.equals("") && gamepad1.x) {
                currentMode = "X";
            } else if (currentMode.equals("") && gamepad1.y) {
                currentMode = "Y";
            }

            switch (currentMode) {
                case "A":
                    // Execute the next step in the A sequence
                    if (step == 0) {
                        robot.drive(2 * robot.feet, 0, .25);
                    } else if (step == 1) {
                        robot.drive(2 * robot.feet, 0, .25);
                    } else if (step == 2) {
                        robot.drive(2 * robot.feet, 0, .25);
                    } else if (step == 3) {
                        robot.drive(0, 2 * robot.feet, .25);
                    } else if (step == 4) {
                        robot.drive(-2 * robot.feet, 0, .25);
                        currentMode = ""; // Sequence finished, reset mode
                        step = -1; // Reset step counter
                    }
                    step++;
                    break;

                case "B":
                    // Execute the next step in the B sequence
                    if (step == 0) {
                        robot.drive(0, -2 * robot.feet, .25);
                    } else if (step == 1) {
                        robot.drive(2 * robot.feet, 0, .25);
                    } else if (step == 2) {
                        robot.drive(2 * robot.feet, 0, .25);
                        currentMode = ""; // Sequence finished, reset mode
                        step = -1; // Reset step counter
                    }
                    step++;
                    break;

                case "X":
                    // Execute the next step in the X sequence
                    if (step == 0) {
                        robot.drive(0, 2 * robot.feet, .25);
                    } else if (step == 1) {
                        robot.drive(2 * robot.feet, 0, .25);
                    } else if (step == 2) {
                        robot.turn(-180);
                        currentMode = ""; // Sequence finished, reset mode
                        step = -1; // Reset step counter
                    }
                    step++;
                    break;

                case "Y":
                    // Execute the next step in the Y sequence
                    if (step == 0) {
                        robot.turn(90);
                    } else if (step == 1) {
                        robot.drive(2 * robot.feet, 0, .25);
                    } else if (step == 2) {
                        robot.drive(2 * robot.feet, 0, .25);
                    } else if (step == 3) {
                        robot.drive(-2 * robot.feet, 0, .25);
                    } else if (step == 4) {
                        robot.turn(-90);
                        currentMode = ""; // Sequence finished, reset mode
                        step = -1; // Reset step counter
                    }
                    step++;
                    break;

                default:
                    break;
            }
            sleep(3000);
            telemetry.update();
        }
    }
}
