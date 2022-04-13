package org.firstinspires.ftc.teamcode.Auto.Good;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.LewaHardware;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import java.util.List;

@Autonomous
public class ServoTest extends AutoBase {
    LewaHardware robot = new LewaHardware();
    public void runOpMode(){
        startUp();
        waitForStart();
        sliderLift(0.5, 3.0, 200.0);
        robot.flippyBox.setPosition(.95);
        telemetry.addData("box position",robot.flippyBox.getPosition());
        telemetry.update();
        sleep(2000);
        robot.flippyBox.setPosition(.4);
        telemetry.addData("box position",robot.flippyBox.getPosition());
        telemetry.update();
        sleep(2000);
        robot.flippyBox.setPosition(.8);
        telemetry.addData("box position",robot.flippyBox.getPosition());
        telemetry.update();
        sleep(2000);
        robot.flippyBox.setPosition(.3);
        telemetry.addData("box position",robot.flippyBox.getPosition());
        telemetry.update();
        sleep(2000);
        robot.flippyBox.setPosition(robot.flippyBox.getPosition()+.65);
        telemetry.addData("box position",robot.flippyBox.getPosition());
        telemetry.update();
        sleep(2000);
        robot.flippyBox.setPosition(robot.flippyBox.getPosition()-.5);
        telemetry.addData("box position",robot.flippyBox.getPosition());
        telemetry.update();
        sleep(2000);
    }

}
