package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SensorSet.LEDMethods;

//TODO: Uncomment the following line to use
@TeleOp
public class CompetitionDriving2024 extends LinearOpMode {

    public static DcMotor motorBR, motorBL, motorFL, motorFR, BackIntake, MiddleIntake, Lift;
    public static CRServo IntakeString;
    private AutonMethods robot = new AutonMethods();
    public int driveswitch = 1;

    public int intakemode = 0;
    private int liftLimit = 5500;

    public void TelemetryUpdate() {
        telemetry.addData("Drive Mode", driveswitch);
        telemetry.addLine();
        telemetry.addData("Intake Mode", intakemode);
        telemetry.addLine();
        telemetry.addData("Servo Position", IntakeString.getPower());
        telemetry.update();

        telemetry.update();
    }
    @Override
    public void runOpMode() {
        //region hardware map
        LEDMethods LED = new LEDMethods();
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        BackIntake = hardwareMap.get(DcMotor.class, "BackIntake");
        MiddleIntake = hardwareMap.get(DcMotor.class, "MiddleIntake");

        IntakeString = hardwareMap.get(CRServo.class, "IntakeString");

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MiddleIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        MiddleIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            TelemetryUpdate();
            double speed = 1;
            if (driveswitch == 0) {
                speed = 1;
            } else if (driveswitch == 1) {
                speed = .66;
            } else if (driveswitch == 2) {
                speed = .333;
            }

            if (gamepad1.a && driveswitch<2) {
                driveswitch +=1;
            }
            else if (gamepad1.b && driveswitch>0) {
                driveswitch -=1;
            }

            if (gamepad1.dpad_up && intakemode<1) {
                intakemode +=1;
            }
            else if (gamepad1.dpad_down && intakemode>-1) {
                intakemode -=1;
            }
            else if (gamepad1.dpad_left) {
                intakemode = 0;
                driveswitch = 1;
            }


            motorFL.setPower(((this.gamepad1.right_stick_y) - (this.gamepad1.right_stick_x) + ((this.gamepad1.left_stick_y)) - (this.gamepad1.left_stick_x)) * speed*.67);
            motorBL.setPower(-(-(this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) - (this.gamepad1.left_stick_y) - (this.gamepad1.left_stick_x)) * speed);
            motorBR.setPower((-(this.gamepad1.right_stick_y) - (this.gamepad1.right_stick_x) - (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * speed);
            motorFR.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * speed*.67);


            BackIntake.setPower(intakemode);
            MiddleIntake.setPower(-1*intakemode);
            if (gamepad1.right_bumper) {
                IntakeString.setPower(0.5);
            }
            else if (gamepad1.left_bumper) {
                IntakeString.setPower(-0.5);
            }
            else {
                IntakeString.setPower(0.0);
            }

            //LIFT
            if (gamepad1.dpad_up && Lift.getCurrentPosition() <= liftLimit) {
                Lift.setPower(1);
            } else if (gamepad1.dpad_down && Lift.getCurrentPosition() >= 500){ //At 500 b/c motor will overspin w/ momentum and end up <0
                Lift.setPower(-1);
            } else {
                Lift.setPower(0);
            }

            telemetry.addData("Lift Pos = ", Lift.getCurrentPosition());
            telemetry.update();
            }
        }
    }




