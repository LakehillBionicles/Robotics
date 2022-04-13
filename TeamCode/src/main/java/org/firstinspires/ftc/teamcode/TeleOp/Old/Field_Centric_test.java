package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LewaHardware;


@TeleOp

@Disabled

public class Field_Centric_test extends LinearOpMode{
    LewaHardware robot = new LewaHardware();
    BNO055IMU imu;
    Orientation angles;
    double startHeading;

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robot.init(hardwareMap);

        //ALREADY IN HWMAP????
        robot.fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bsd.setDirection(DcMotorSimple.Direction.REVERSE);


        robot.fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status:", "Run time dudes");
        telemetry.update();

        waitForStart();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        startHeading = angles.firstAngle;

        while (opModeIsActive()) {
            drive();
            spin();
        }

    }



    public void drive() //field centered drive
    {
        if (gamepad1.left_stick_y < -.25 || gamepad1.left_stick_y > .25 || gamepad1.left_stick_x < -.25 || gamepad1.left_stick_x > .25)
        {
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_y,2)+Math.pow(gamepad1.left_stick_x,2));
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double currentHeading = angles.firstAngle;


            robot.fsd.setPower((Math.sin(-startHeading+currentHeading+angle-Math.PI/4))*magnitude);
            robot.bpd.setPower((Math.sin(-startHeading+currentHeading+angle-Math.PI/4))*magnitude);
            robot.fpd.setPower((Math.sin(-startHeading+currentHeading+angle+Math.PI/4))*magnitude);
            robot.bsd.setPower((Math.sin(-startHeading+currentHeading+angle+Math.PI/4))*magnitude);

        }
        else{
            robot.fpd.setPower(0.0);
            robot.bpd.setPower(0.0);
            robot.fsd.setPower(0.0);
            robot.bsd.setPower(0.0);

        }
    }

    public void spin(){

        if(gamepad1.right_stick_x > 0.2 && gamepad1.right_stick_y > -0.3 && gamepad1.right_stick_y < 0.3){    //clockwise
            robot.fpd.setPower(-gamepad1.right_stick_x);
            robot.bpd.setPower(-gamepad1.right_stick_x);
            robot.fsd.setPower(gamepad1.right_stick_x);
            robot.bsd.setPower(gamepad1.right_stick_x);


        }else if(gamepad1.right_stick_x < -0.2 && gamepad1.right_stick_y > -0.3  && gamepad1.right_stick_y < 0.3 ){    //counterclockwise
            robot.fpd.setPower(-gamepad1.right_stick_x);
            robot.bpd.setPower(-gamepad1.right_stick_x);
            robot.fsd.setPower(gamepad1.right_stick_x);
            robot.bsd.setPower(gamepad1.right_stick_x);

        }

    }
}
