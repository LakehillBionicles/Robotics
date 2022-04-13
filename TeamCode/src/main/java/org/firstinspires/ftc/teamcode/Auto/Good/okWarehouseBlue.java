package org.firstinspires.ftc.teamcode.Auto.Good;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
//@Disabled
//start near warehouse, score goal, go through gap to park in warehouse
public class okWarehouseBlue extends AutoBase {

    public void runOpMode () {

        startUp();
        waitForStart();
        String duckPosition = detectDuck();

        encoderDrive(.6,10,10,3); //made slower & further
        encoderDrive(.5,-12.75,12.75,4); //changed to slower, bigger turn
        encoderDrive(.6,25,25,4); //made slower & further
        //encoderDrive(.8,4,4,4);                               //commented out
        liftFreight(duckPosition);
        sleep(1600);
        resetArm();
        //lowered by 50
        encoderDrive(.3,-14,-14,2);
        turn45Left();
        encoderDrive(.5, 7.0, 7.0, 2.0); //made this slightly smaller
        //sideways(.5,-8.5,-8.5,5);                                //commented this out
        //turn90Left();
        encoderDrive(.6,-18,18,3); //changed to one bigger, faster turn
        encoderDrive(1.0,-12,-12,5); //back into warehouse
        colorSenseFar();
        //colorSense()
        //sideways(.5,25,25,3); //should go right, strafe into wall
        //        encoderDrive(.7,-47,-47,5); //back into warehouse

        stop();
    }
    }


