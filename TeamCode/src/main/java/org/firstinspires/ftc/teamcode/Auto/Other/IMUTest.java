package org.firstinspires.ftc.teamcode.Auto.Other;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.LewaHardware;


@Autonomous

@Disabled

public class IMUTest extends AutoBase {
    LewaHardware robot = new LewaHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status:", "Run time dudes");
        telemetry.update();
        waitForStart();
        startUp();

        while (opModeIsActive()) {
            updateAngles();
            turn(90,.5); //turn left 90 degrees
            turn(-180,.5); //turn right 180
            stop();

        }

    }
}
