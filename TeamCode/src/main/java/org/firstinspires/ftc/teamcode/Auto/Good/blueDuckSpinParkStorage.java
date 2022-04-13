package org.firstinspires.ftc.teamcode.Auto.Good;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
//@Disabled

// spin duck, park in storage
public class blueDuckSpinParkStorage extends AutoBase {

        public void runOpMode() {
            startUp();
            waitForStart();
            String duckPosition = detectDuck();


            encoderDrive(.5, 3.0, 3.0, 2.0);
            turn90Right();

            encoderDrive(.3,15.0,15.0,2.0);
            spinCarousel();
            sleep(2000);
            encoderDrive(.5,-12.0,-12.0, 2.0);
            sideways(.5,10.0,10.0,2);
            encoderDrive(.5, 25, 25, 2);
            resetArm();
            stop();
        }

}

