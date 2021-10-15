package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LewaHardware;


@TeleOp

//@Disabled

public class NewBaseTest extends LinearOpMode {
    LewaHardware robot = new LewaHardware();
    public DcMotor pd = null;
    public DcMotor sd = null;
    HardwareMap hwMap = null;


    public void runOpMode() {
        init3(hardwareMap);

        telemetry.addData("Status:", "Run time dudes");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive();
        }

    }


    public void drive() {
        if (gamepad1.right_stick_y > 0.1) {
            sd.setPower(gamepad1.right_stick_y);
        } else if (gamepad1.right_stick_y < -.1) {
            sd.setPower(gamepad1.right_stick_y);
        } else {
            sd.setPower(0);
        }
        if (gamepad1.left_stick_y > 0.1) {
            pd.setPower(-gamepad1.left_stick_y);
        } else if (gamepad1.left_stick_y < -.1) {
            pd.setPower(-gamepad1.left_stick_y);
        } else {
            pd.setPower(0);
        }
        if (gamepad1.a){
            pd.setPower(1);
            sd.setPower(-1);
        }else{
            pd.setPower(0);
            sd.setPower(0);
        }

    }

    public void init3(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        pd = hwMap.get(DcMotor.class, "pd");
        sd = hwMap.get(DcMotor.class, "sd");


        //set direction of rotations
        pd.setDirection(DcMotorSimple.Direction.FORWARD);
        sd.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set all motors to zero power
        pd.setPower(0);
        sd.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        pd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ///////////////////////

        pd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
