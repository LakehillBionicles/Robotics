package org.firstinspires.ftc.teamcode.Auto.Good;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
//@Disabled
//start at storage, score goal, spin duck, park in warehouse
public class okBlueDuckSpin extends AutoBase {
    public void runOpMode () {

        startUp();
        waitForStart();
        String duckPosition = detectDuck();

        encoderDrive(.7,17.3,17.3,2);
        encoderDrive(.7,10,-10,2);
        encoderDrive(.7,19.18,19.18,2);//go to hub
        liftFreight(duckPosition);
        sleep(1650);
        resetArm();
        encoderDrive(.25,-30,-30,2);
        encoderDrive(.9, -36,36,3);//turn 135 degrees left to spinner
        encoderDrive(.12, 32.2, 32.2, 4);

        //encoderDrive(.3,3,3,2);
        spinCarousel();
        sleep(900);
        encoderDrive(.45,-16,-12,2);//back away from duck spinner
        //encoderDrive(.5,-60,-60,4);
        //sideways(.5,-15,-15,2); //should go right
        colorSense();//drive into warehouse with color sensor
        sideways(.7,-16,-20,2);
        //encoderDrive(.7,-47,-47,3); //back into warehouse
        //sideways(.5,-25,-25,3); //should go right, strafe into wall
        //        encoderDrive(.7,-47,-47,5); //back into warehouse
        stop();
    }
}
