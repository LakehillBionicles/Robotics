package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous
public class JustParkBlue extends AutoBase {

    public void runOpMode() {
        startUp();
        waitForStart();
        while (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1, 16.0 / 9.0);
            }
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            float leftBound = 0;
            if (updatedRecognitions != null) {
                telemetry.addData("Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    leftBound = recognition.getLeft();
                    i++;
                }

                if (leftBound > 850 && leftBound < 900)  //duck closest to alliance storage
                {
                    telemetry.addData("Duck: ", "3 (duck closest to alliance storage)");
                } else if (leftBound > 110 && leftBound < 210)    //duck in middle
                {
                    telemetry.addData("Duck: ", "2 (duck in middle)");
                } else    //duck closest to shipping hub
                {
                    telemetry.addData("Duck: ", "1 (duck closest to shipping hub)");
                }
                telemetry.update();
                park();
                stop();
            }

        }

    }
        public void park()
        {
            sideways(.5, -24, -24, 10);
            encoderDrive(.7, 96, 96, 10);
        }

    }

