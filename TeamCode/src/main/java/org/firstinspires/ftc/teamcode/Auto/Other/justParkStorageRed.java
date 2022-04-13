package org.firstinspires.ftc.teamcode.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

@Autonomous
@Disabled

public class justParkStorageRed extends AutoBase {
    public void runOpMode () {

        startUp();
        waitForStart();

        encoderDrive(.7, 6, 6, 5);
        turn90Right();
        encoderDrive(1,28,28,5);
        turn90Left();
        encoderDrive(1,8,8,4);
        turn90Right();
        encoderDrive(.7,44,44,4);

        stop();

    }
}
