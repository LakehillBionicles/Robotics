package org.firstinspires.ftc.teamcode.Auto.Good;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
//@Disabled



public class redDuckSpinParkStorage extends AutoBase {
    public void runOpMode() {
        startUp();
        waitForStart();
        String duckPosition = detectDuck();


        encoderDrive(.5, 12.0, 12.0, 2.0);
        turn(-90,.2);
        encoderDrive(.3,15.0,15.0,2.0);
        turn(-180,.2);
        encoderDrive(.2, 10.3,10.3, 2.0);
        turn45Right();
        encoderDrive(.2,10,10,2.0);
        spinCarouselRed();
        sleep(2000);
        turn45Left();
        encoderDrive(.5,-12.0,-12.0, 2.0);
        resetArm();
        stop();
    }
}
