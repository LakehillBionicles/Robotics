package org.firstinspires.ftc.teamcode.TeleOp.Old;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LewaHardware;
import org.firstinspires.ftc.teamcode.LewaHardware;

@TeleOp

//@Disabled

public class TestMotors extends LinearOpMode {

    LewaHardware robot =  new LewaHardware();
    BNO055IMU imu;
    Orientation angles;

    public DcMotor testMotor1 = null;
    public DcMotor testMotor2 = null;
    public DcMotor testMotor3 = null;
    public DcMotor testMotor4 = null;
    public ColorSensor color = null;
    HardwareMap hwMap = null;
    boolean enter = false;
    public static double testMotorSpeed = 1.0;
    int turn = 0;

    public void runOpMode(){

        init2(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        waitForStart();
        while(opModeIsActive()){
            //instructions();
            //testMotor();
            displayIMU();
            //fakeTurn(90);
            //displayColor();
        }
    }


    public void displayIMU(){
        if (gamepad1.left_bumper)
        {
            //imu.close();
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            imu.initialize(parameters);
        }
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("1st Angle",angles.firstAngle); //yaw --> 2d movement (zyx)
        telemetry.addData("2nd Angle",angles.secondAngle); //pitch
        telemetry.addData("3rd Angle",angles.thirdAngle); //roll
        if (turn == 1)
        {
            telemetry.addData("turning","");
        }
        if (turn == 2)
        {
            telemetry.addData("finished turn","");
            turn = 0;
        }
        telemetry.update();
    }

    public void displayColor(){
        telemetry.addData("alpha",color.alpha());
        telemetry.addData("red",color.red());
        telemetry.addData("green",color.green());
        telemetry.addData("blue",color.blue());
        telemetry.update();

    }
    public void testMotor(){
    if (gamepad1.dpad_down){
    testMotorSpeed= -Math.abs(testMotorSpeed);
    }
    else if (gamepad1.dpad_up)
    {
        testMotorSpeed = Math.abs(testMotorSpeed);
    }
        if(gamepad1.dpad_left) {
            if (testMotorSpeed >= 0) {
                testMotorSpeed -= 0.02;
            } else if (testMotorSpeed < 0) {
                testMotorSpeed += 0.02;
            }
        }
    else if(gamepad1.dpad_right) {
            if (testMotorSpeed >= 0) {
                testMotorSpeed += 0.02;
            } else if (testMotorSpeed < 0) {
                testMotorSpeed -= 0.02;
            }
        }
        if(gamepad1.y){
            testMotor1.setPower(testMotorSpeed);
        } else {
            testMotor1.setPower(0.0);
        }

        if(gamepad1.b){
            testMotor2.setPower(testMotorSpeed);
        } else {
            testMotor2.setPower(0.0);
        }

        if(gamepad1.a){
            testMotor3.setPower(testMotorSpeed);
        } else {
            testMotor3.setPower(0.0);
        }

        if(gamepad1.x){
            testMotor4.setPower(testMotorSpeed);
        } else {
            testMotor4.setPower(0.0);
        }


    }
    public void displayInfo(){
        telemetry.addData("Speed",testMotorSpeed);
        telemetry.update();
    }
    public void instructions()
    {
        if (gamepad1.a || gamepad1.x || gamepad1.y || gamepad1.b || gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right)
        {
            enter = true;
        }
        if (!enter)
        {
            telemetry.addData("Use left/right to adjust motor speed","");
            telemetry.addData("Use up/down to change motor direction","");
        }
    }
    public void init2(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        testMotor1 = hwMap.get(DcMotor.class, "testMotor1");
        testMotor2 = hwMap.get(DcMotor.class, "testMotor2");
        testMotor3 = hwMap.get(DcMotor.class, "testMotor3");
        testMotor4 = hwMap.get(DcMotor.class, "testMotor4");
        color = hwMap.get(ColorSensor.class, "color");



        //set direction of rotations
        testMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor4.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        testMotor3.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        testMotor1.setPower(0);
        testMotor3.setPower(0);
        testMotor2.setPower(0);
        testMotor4.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ///////////////////////

        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   //new, could mess soomrthing up, but should be fine
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}

  /* need to create a new config for this
  plug hub into battery
  declare port 0 for testMotor1 then should be good to good
  left trigger = forward (positive hopefully)
  right trigger = backward (negative hopefully)
  make sure we don't need to update phone as well


  */