package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LewaHardware;

@TeleOp

public class LewaDriveSprings extends DriveBaseSprings{
    LewaHardware robot = new LewaHardware();
    int mode, driveMode = 0;

    BNO055IMU imu;
    Orientation angles;



    //@Disabled

    @Override
    public void runOpMode() {
        super.runOpMode();
        startUpFast();
        waitForStart();

        while (opModeIsActive()) {
            //drive();
            driveTank();
            //spin();
            //theAbyss();
            slowMode();
            intake();
            armSlider();
            duckSpin();
            flipBox();
            //displayLocation();
            testInfo();
            doLights();
        }

    }
    public void slowMode() {super.slowMode(gamepad1.dpad_down, gamepad1.dpad_right);}

    public void intake() {super.intake(gamepad1.left_trigger,gamepad1.right_trigger);}

    public void armSlider() {super.armSlider(gamepad1.left_bumper, gamepad1.right_bumper);}

    public void flipBox() {super.flipBox(gamepad1.y,gamepad1.a);}

    public void duckSpin() {super.duckSpin(gamepad1.x,gamepad1.b);}

    public void drive() {super.drive(gamepad1.left_stick_x, gamepad1.left_stick_y);}

    public void spin() {super.spin(gamepad1.right_stick_x);}

    public void driveTank() {super.driveTank(gamepad1.left_stick_y, gamepad1.right_stick_y);}

    public void drive2() {super.drive2(gamepad1.left_stick_x, gamepad1.left_stick_y);}

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
    public void driveFieldCentric() {super.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y);}

}


/*toggleModes();
            if (mode == 0) {
                driveFieldCentric();
            }
            else if (mode == 1) {
                drive();
            }
             */
