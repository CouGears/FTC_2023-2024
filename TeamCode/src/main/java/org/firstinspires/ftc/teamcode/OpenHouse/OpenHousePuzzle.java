package org.firstinspires.ftc.teamcode.OpenHouse;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SensorSet.LEDMethods;

//TODO: Uncomment the following line to use
@TeleOp
@Disabled
public class OpenHousePuzzle extends OpMode {

    private AutonMethods robot = new AutonMethods();
    private ElapsedTime runtime = new ElapsedTime();
    double rev = 537.7; //312 rpm motor
    double inch = rev / (3.78 * 3.14);
    double feet = inch * 12 + (10 * inch);

    HardwareMap map;
    Telemetry tele;

    @Override
    public void init() {
        runtime.reset();
        robot.init(hardwareMap, telemetry, false);
    }

    @Override
    public void loop() {
        /*if (gamepad1.a) {
            robot.drive(3*feet, 0,.25);
            robot.drive(0, 1*feet,.25);
            robot.drive(-1*feet, 0,.25);
        }
        else if (gamepad1.b) {
            robot.drive(0, -1*feet,.25);
            robot.drive(2*feet, 0,.25);
        }
        else if (gamepad1.x) {
            robot.drive(0, 1*feet,.25);
            robot.drive(1*feet, 0,.25);
            robot.turn(180);
        }
        else if (gamepad1.y) {
            robot.turn(-90);
            robot.drive(2*feet, 0,.25);
            robot.drive(-1*feet, 0,.25);
            robot.turn(90);
        }*/
        if (gamepad1.a) {
            robot.drive(2*feet, 0,.2);
            robot.newSleep(.75);
            robot.speed(0);
        }
        else if (gamepad1.b) {
            robot.drive(-2*feet, 0,.2);
            robot.newSleep(.75);
            robot.speed(0);
        }
        else if (gamepad1.x) {
            robot.turn(20, .2);
            robot.newSleep(.75);
            robot.speed(0);
        }
        else if (gamepad1.y) {
            robot.turn(-20, .2);
            robot.newSleep(.75);
            robot.speed(0);
        }
    }
}





