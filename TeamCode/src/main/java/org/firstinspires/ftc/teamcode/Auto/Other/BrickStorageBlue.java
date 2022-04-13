package org.firstinspires.ftc.teamcode.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
@Disabled

public class BrickStorageBlue extends AutoBase {
    public void runOpMode(){
        startUp();
        waitForStart();
        String duckPosition = detectDuck();

        turn(-90,.5);
        encoderDrive(.8,30,30,2);
        turn(90,.5);
        encoderDrive(.8,5,5,2);
        liftFreight(duckPosition);
        turn(125,.5); //turn left to face carousel
        encoderDrive(.8,28,28,2);
        spinCarousel();
        turn(-125,.5); //same angle as line 17;
        encoderDrive(.8,72,72,4);
        turn(-90,.5);
        encoderDrive(.8,24,24,2 );
        turn(90,.5);
        encoderDrive(.7,48,48,8 ); //going over barrier
        stop();

    }
}
