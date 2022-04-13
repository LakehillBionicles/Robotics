package org.firstinspires.ftc.teamcode.Auto.Other;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
@Disabled

public class WarehouseBlue extends AutoBase {
    @Override
    public void runOpMode () {

        startUp();
        waitForStart();
        String duckPosition = detectDuck();

        encoderDrive(.7,10,10,3);
        encoderDrive(.7,8.5,-8.5,4);
        encoderDrive(.7,17.5,17.5,4);
        liftFreight(duckPosition);
        sleep(1650);
        encoderDrive(.3,-16,-16,2);
        turn45Right();
        turn45Right();
        encoderDrive(.7, -9,9,4);
        encoderDrive(.5, 20.0, 20.0, 2.0); //go to duck spinner
        encoderDrive(.3,3,3,2.0);
        spinCarousel();
        sleep(1000);
        encoderDrive(.5,-60,-60,3);
        sideways(.5,-15,-15,3); //should go right
        encoderDrive(.7,-47,-47,5); //back into warehouse
        //sideways(.5,-25,-25,3); //should go right, strafe into wall
        //        encoderDrive(.7,-47,-47,5); //back into warehouse
        stop();
    }
}