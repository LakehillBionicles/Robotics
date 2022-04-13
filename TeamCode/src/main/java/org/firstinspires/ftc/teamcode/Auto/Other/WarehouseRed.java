package org.firstinspires.ftc.teamcode.Auto.Other;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;


@Autonomous
@Disabled

public class WarehouseRed extends AutoBase {
           @Override
        public void runOpMode() {
            startUp();
            waitForStart();
            String duckPosition = detectDuck();

               detectDuck();
// Define function later

               encoderDrive(0.8, 6.0, 6.0, 5.0);
               turn(-135, 0.5);


//turnRight();

               //liftFreightRed(duckPosition);
               //liftFreightRed(detectDuckRed());

               turn(45, 0.5); //could change based on slider arm orientation - set for facing towards hub

        }
}
