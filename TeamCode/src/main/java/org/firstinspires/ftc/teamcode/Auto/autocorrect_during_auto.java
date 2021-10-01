package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LewaHardware;


@Autonomous(name="autocorrect during auto")

//@Disabled

public class autocorrect_during_auto extends AutoBase {

    LewaHardware robot = new LewaHardware();
    BNO055IMU imu;
    Orientation angles;
    double startHeading;
    final double pi = 3.14159265359;
    @Override

    public void runOpMode() {

        startUp();

        waitForStart();

        while (opModeIsActive()) {


            encoderDrivePlus(.25, 50, 50, 10);

            stop();////////////////////////////////


        }
    }
}
