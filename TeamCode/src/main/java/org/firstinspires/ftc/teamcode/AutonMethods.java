package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutonMethods{

    //Constructor
    public AutonMethods() {

    }

    //Declare and initial variables
    private double rev = 537.7;//revolution of 312 rpm motor , find at https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-6mm-d-shaft-312-rpm-36mm-gearbox-3-3-5v-encoder/  called encoder resolution
    private double pi = 3.14;
    private double wheelDiameter = 3.77953;//inch
    private double robotWidth = 12.75;//inch
    private double robotLength = 13;//inch
    private double CalebTurnConstant = .9;
    private double CalebDistanceConstant = .69;

    private double circumscribedDiameter = Math.sqrt(Math.pow(robotLength, 2) + Math.pow(robotWidth, 2));//inch
    private double circumscribedRadius = circumscribedDiameter / 2;//inch
    private double inch = rev / (wheelDiameter * pi);
    private double feet = inch * 12;
    private double rev2 = 2048;//revolution of 435 rpm motor
    private double inch2 = rev2 / (2 * pi);
    private double feet2 = inch2 * 12;

    private int topLiftEncoder = 7475;
    private double botR = 1;
    private double topR = 0;
    private double botL = .35;
    private double topL = 0;
    private double FRtpos, BRtpos, FLtpos, BLtpos;
    public static DcMotor motorBR, motorBL, motorFL, motorFR, LiftRight, LiftLeft;
    //public static DcMotor Forwards = intake, Sideways = carousel;
    public static Servo intake, armL, armR;
    public static DistanceSensor distanceSensor, distanceSensorBack;
    // public static LED red, green, red2, green2;
    public TouchSensor armTouch;
    private final ElapsedTime runtime = new ElapsedTime();
    public static int Case = 0;
    HardwareMap map;
    Telemetry tele;
    public int counter = 0;

    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    //Initialization
    public void setIntakePOS(double a) {
        intake.setPosition(a);
    }

    public void initBasic(HardwareMap map, Telemetry tele) {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");

        motorFL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
    }
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        LiftRight = map.get(DcMotor.class, "LiftRight");
        LiftLeft = map.get(DcMotor.class, "LiftLeft");
        // release = map.get(DcMotor.class, "release");

       /* red = map.get(LED.class, "red");
        green = map.get(LED.class, "green");
        red2 = map.get(LED.class, "red2");
        green2 = map.get(LED.class, "green2");*/

        intake = map.get(Servo.class, "intake");


        motorFL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        LiftLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        LiftRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        //  release.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // release.setDirection(DcMotorSimple.Direction.FORWARD);

        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
        LiftLeft.setTargetPosition(0);
        LiftRight.setTargetPosition(0);

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

         tele.addData(">", "Gyro Calibrating. Do Not Move!");
        tele.update();
    }

    public void kill() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        LiftLeft.setPower(0);
        LiftRight.setPower(0);

    }

    public double maps(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    //Function to move the robot in any direction
    public void drive(double forward, double sideways, double speed) {
        runtime.reset();
        forward*=CalebDistanceConstant;
        sideways*=CalebDistanceConstant;
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 2) break;
        }
        motorFL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(RunMode.STOP_AND_RESET_ENCODER);

        FRtpos = forward - sideways;
        BRtpos = forward + sideways;
        FLtpos = forward + sideways;
        BLtpos = forward - sideways;

        motorFL.setTargetPosition(-(int) FLtpos);
        motorBL.setTargetPosition((int) BLtpos);
        motorFR.setTargetPosition(-(int) FRtpos);
        motorBR.setTargetPosition((int) BRtpos);

        motorFL.setMode(RunMode.RUN_TO_POSITION);
        motorBL.setMode(RunMode.RUN_TO_POSITION);
        motorFR.setMode(RunMode.RUN_TO_POSITION);
        motorBR.setMode(RunMode.RUN_TO_POSITION);

        speed(speed);
    }
    public void speed(double spee) {
        motorFL.setPower(spee);
        motorBL.setPower(spee);
        motorFR.setPower(spee);
        motorBR.setPower(spee);
    }


    //circumscribed robot has a diameter of 21 inches
    public void turn(double deg) {
        deg *= CalebTurnConstant;
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 2) break;
        }
        motorFL.setMode(RunMode.STOP_AND_RESET_ENCODER); //for every drive function remember to reset encoder
        motorBL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        double deltaturn = (deg / 360.0) * circumscribedDiameter * pi * inch;
        motorFL.setTargetPosition(-(int) deltaturn);
        motorBL.setTargetPosition((int) deltaturn);
        motorFR.setTargetPosition((int) deltaturn);
        motorBR.setTargetPosition(-(int) deltaturn);
        motorFR.setMode(RunMode.RUN_TO_POSITION);
        motorBR.setMode(RunMode.RUN_TO_POSITION);
        motorFL.setMode(RunMode.RUN_TO_POSITION);
        motorBL.setMode(RunMode.RUN_TO_POSITION);
        motorFL.setPower(0.5);
        motorBL.setPower(0.5);
        motorFR.setPower(0.5);
        motorBR.setPower(0.5);

    }

    public void LiftSetPosition(int position) {
        LiftLeft.setTargetPosition(position);
        LiftRight.setTargetPosition(position);
        LiftRight.setMode(RunMode.RUN_TO_POSITION);
        LiftLeft.setMode(RunMode.RUN_TO_POSITION);
        LiftLeft.setPower(1);
        LiftRight.setPower(1);
    }
    public void LiftArmSetPosition(int position) {
        LiftLeft.setTargetPosition(position);
        LiftRight.setTargetPosition(position);
        LiftRight.setMode(RunMode.RUN_TO_POSITION);
        LiftLeft.setMode(RunMode.RUN_TO_POSITION);
        LiftLeft.setPower(1);
        LiftRight.setPower(1);
        double left = maps(LiftLeft.getCurrentPosition(), 0, topLiftEncoder, botL, topL);
        double right = maps(LiftLeft.getCurrentPosition(), 0, topLiftEncoder, botR, topR);
        armL.setPosition(left);
        armR.setPosition(right);
    }
    public int LiftGetPosition() {
        int leftPosition = LiftLeft.getCurrentPosition();
        return (leftPosition);
    }

//

    public void newSleep(double timeinSeconds) {
        runtime.reset();
        while (runtime.seconds() < timeinSeconds) ;
//do nothing
    }

    //Function to have the robot sleep
    public void sleep(long sleep) {
        try {
            Thread.sleep(sleep);
        } catch (InterruptedException e) {
            tele.addLine("Failed Sleep");
            tele.update();
        }
    }
    public void lift(double amount) { //moves the 4 bar/lifter
        // amount = -amount;
        LiftRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        LiftLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        LiftRight.setTargetPosition((int) amount);
        LiftLeft.setMode(RunMode.RUN_TO_POSITION);
        LiftRight.setPower(.6);
        LiftLeft.setPower(.6);
    }
    public void dump()
    {
        intake.setPosition(-.25);
    }


}