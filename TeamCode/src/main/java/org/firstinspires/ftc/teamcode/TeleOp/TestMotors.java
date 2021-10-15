package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LewaHardware;
import org.firstinspires.ftc.teamcode.LewaHardware;

@TeleOp

//@Disabled

public class TestMotors extends LinearOpMode {

    LewaHardware robot =  new LewaHardware();

    public DcMotor testMotor1 = null;
    public DcMotor testMotor2 = null;
    public DcMotor testMotor3 = null;
    public DcMotor testMotor4 = null;
    HardwareMap hwMap = null;
    public static final double testMotorSpeed = 1.0;

    public void runOpMode(){

        init2(hardwareMap);

        telemetry.addData("Status:", "Run");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            testMotor();
        }
    }




    public void testMotor(){

        if(gamepad1.y){                                 //positive??
            testMotor1.setPower(testMotorSpeed);
        } else {
            testMotor1.setPower(0.0);
        }

        if(gamepad1.b){                                 //positive??
            testMotor2.setPower(testMotorSpeed);
        } else {
            testMotor2.setPower(0.0);
        }

        if(gamepad1.a){                                 //negative??
            testMotor1.setPower(-(testMotorSpeed));
        } else {
            testMotor1.setPower(0.0);
        }

        if(gamepad1.x){                                 //negative??
            testMotor2.setPower(-(testMotorSpeed));
        } else {
            testMotor2.setPower(0.0);
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