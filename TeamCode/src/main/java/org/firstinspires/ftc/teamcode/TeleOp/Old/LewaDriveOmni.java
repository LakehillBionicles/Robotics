package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LewaHardware;
import org.firstinspires.ftc.teamcode.TeleOp.Old.DriveBaseOmni;

@TeleOp
@Disabled
public class LewaDriveOmni extends DriveBaseOmni {
    LewaHardware robot = new LewaHardware();
    int mode = 1;
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        super.runOpMode();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robot.init(hardwareMap);

        telemetry.addData("Status:", "Run time dudes");
        telemetry.update();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        startUp();
        waitForStart();

        while (opModeIsActive()) {
            toggleModes();
            if (mode == 1) {
                driveWithIMU();
            }
            else if (mode == 2)
            {
                driveTank();
            }
            else if (mode == 0)
            {
                drive();
            }
            slowMode();
            spin();
            intake();
            armSlider();
            duckSpin();
            flipBox();
            testDistance();
        }

    }


    public void toggleModes() {
        if (gamepad1.dpad_up) //imu
        {
            mode = 1;
            telemetry.addData("Mode","imu");
        }
        else if (gamepad1.dpad_left) //no imu
        {
            mode = 0;
            telemetry.addData("Mode","no imu");
        }
        else if (gamepad1.dpad_right) //tank
        {
            mode = 2;
            telemetry.addData("Mode","tank");
        }
        telemetry.update();

    }

    public void slowMode() {super.slowMode(gamepad1.dpad_down);}

    public void drive() {super.drive(gamepad1.left_stick_y);}

    public void driveWithIMU() {super.driveWithIMU(gamepad1.left_stick_y);}

    public void spin() {super.spin(gamepad1.right_stick_x);}

    public void intake() {super.intake(gamepad1.left_trigger,gamepad1.right_trigger);}

    public void armSlider() {super.armSlider(gamepad1.right_bumper, gamepad1.left_bumper);}

    public void flipBox() {super.flipBox(gamepad1.y,gamepad1.a);}

    public void duckSpin() {super.duckSpin(gamepad1.x,gamepad1.b);}

    public void driveTank() {super.driveTank(gamepad1.left_stick_y,gamepad1.right_stick_y);}




}
