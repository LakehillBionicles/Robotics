package org.firstinspires.ftc.teamcode.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
@Disabled

public class okStorageRed extends AutoBase {
    public void runOpMode () {

        startUp();
        waitForStart();
        String duckPosition = detectDuck(); //after dropping the freight, it doesn't back up and takes the goal with it when it goes forward

        //encoderDrive(.7,8,8,3);
       // turn45Right();
        //encoderDrive(.7,10.0,10.0,4);
        //liftFreightBlue(duckPosition);
        //sleep(2000);                                     //increase?
        //encoderDrive(.3,-16.5,-16.5,2); //this doesn't run?
        //sliderLift(.9,5.0,-900);
        //turn45Right();
        //turn45Right();
        //turn90Left();
        //encoderDrive(.3, 11, 11, 4);
        encoderDrive(.7, 4,4,4);
        turn90Right();
        encoderDrive(.7,15,15,4);
        turn90Left();
        encoderDrive(.7,10.,10.,4);
        liftFreight(duckPosition);
        //sleep(2000);                                     //increase?
        encoderDrive(.3,-11,-11,2); //this doesn't run?
        turn90Right();
        encoderDrive(.5, 85, 85, 12);
        //turn90Right();
        //encoderDrive(.3,14.0,14.0,5);
        //turn90Right();
        //encoderDrive(.3,5,5,4);
        //turn90Left();
        //encoderDrive(.4, 15, 15, 5);

        stop();
    }
}
