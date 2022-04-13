package org.firstinspires.ftc.teamcode.Auto.Good;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
//@Disabled


public class okRedDuckSpin extends AutoBase {
    public void runOpMode () {

        startUp();
        waitForStart();
        String duckPosition = detectDuck();
sleep(5000);
        encoderDrive(.7,15,15,2);
        encoderDrive(.5,-13.5,13.5,2);
        encoderDrive(.7,23.65,23.65,2);//go to hub
        liftFreight(duckPosition);
        sleep(1650);
        resetArm();
        encoderDrive(.25,-20,-20,2);
        encoderDrive(.9, 32.5,-32.5,3);//turn 135 degrees right to spinner
        encoderDrive(.25, 28,28, 2);
        encoderDrive(.8,24.5,-24.5,2);//turn 90 degrees right to spinner
        encoderDrive(.12,26.5,26.5,3);
        //encoderDrive(.3,3,3,2);
        spinCarouselRed();
       /* sleep(900);
        encoderDrive(.3, -5,-5,2);
        encoderDrive(.8,-15.5,15.5,2);

        encoderDrive(.45,-12,-6,2);//back away from duck spinner
        //encoderDrive(.5,-60,-60,4);
        //sideways(.5,-15,-15,2); //should go right
        colorSense();//drive into warehouse with color sensor
        sideways(.7,16,20,2);
        //sideways(.7,-16,11,2);
        //encoderDrive(.7,-47,-47,3); //back into warehouse
        //sideways(.5,-25,-25,3); //should go right, strafe into wall
        //        encoderDrive(.7,-47,-47,5); //back into warehouse
        stop();*/
    }
}
