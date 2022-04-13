package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LewaHardware;


@TeleOp

public class LewaDrive_2Springs extends DriveBaseSprings{
    LewaHardware robot = new LewaHardware();
    int mode = 0;
    BNO055IMU imu;
    Orientation angles;

//@Disabled
    @Override
    public void runOpMode() {
        super.runOpMode();
        startUpFast();
        waitForStart();

        while (opModeIsActive()) {
            spin();
            drive();
            theAbyss();
            slowMode();
            intake();
            armSlider();
            duckSpin();
            flipBox();
            doLights();
            testInfo();
        }
    }
    public void drive() {super.drive(gamepad1.left_stick_x, gamepad1.left_stick_y); }

    public void spin() {super.spin(gamepad1.right_stick_x); }

    public void slowMode() {super.slowMode(gamepad1.dpad_down, gamepad1.dpad_right); }

    public void intake() {super.intake(gamepad1.left_trigger, gamepad1.right_trigger); }

    public void armSlider() {super.armSlider(gamepad2.left_stick_y > 0, gamepad2.left_stick_y < 0); }

    public void flipBox() {super.flipBox(gamepad2.a, gamepad2.y); }

    public void duckSpin() {super.duckSpin(gamepad1.right_bumper, gamepad1.left_bumper); }

    //public void driveFieldCentric() {super.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y); }
    public void toggleModes() {
        if (gamepad1.dpad_up) //imu
        {
            mode = 1;
            telemetry.addData("Mode","normal");
        }
        else if (gamepad1.dpad_left) //no imu
        {
            mode = 0;
            telemetry.addData("Mode","Field Centric");
        }
        telemetry.update();

    }

}