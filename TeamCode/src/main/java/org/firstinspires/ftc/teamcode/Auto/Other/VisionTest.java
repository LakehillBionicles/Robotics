
package org.firstinspires.ftc.teamcode.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.LewaHardware;

import java.util.List;

@Autonomous
@Disabled

public class VisionTest extends AutoBase {
    public void runOpMode() {
        startUp();
        waitForStart();
        while (opModeIsActive()) {
            detectDuck();
        }
    }
}
