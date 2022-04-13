package org.firstinspires.ftc.teamcode.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;


@Autonomous
    @Disabled


    public class justParkWarehouseBlue extends AutoBase {

        @Override
        public void runOpMode() {
            startUp();
            waitForStart();
            encoderDrive(1,12.5,12.5,3);
            turn90Left();
            encoderDrive(.7,48,48,4);

            stop();

        }

    }

