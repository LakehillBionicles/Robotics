package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.LewaHardware;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentHashMap;

public class DriveBaseSprings extends AutoBase {
    LewaHardware robot = new LewaHardware();
    double speed = 1;
    double startHeading;
    boolean isMoving;
    BNO055IMU imu;
    Orientation angles;
   // double x, y, t, t2 = 0;
    // ConcurrentHashMap<Double,Double> xa, ya, xv, yv, xp, yp = new ConcurrentHashMap<>();

//add macro to drive so back wheel is in between bars
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robot.init(hardwareMap);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //xa.put(0.0,0.0);
        //ya.put(0.0,0.0);
        //xv.put(0.0,0.0);
        //yv.put(0.0,0.0);
        //xp.put(0.0,0.0);
        //yp.put(0.0,0.0);
    }

    public void intake(double in, double out) {
        if (in > 0) {
            robot.intakeMotor.setPower(1);//intake in
        } else if (out > 0) {
            robot.intakeMotor.setPower(-.6);
        } else {
            robot.intakeMotor.setPower(0);
        }
    }

    public void armSlider(boolean up, boolean down) {
        if (up) {
            robot.sliderVertical.setPower(.75);//arm is supposed to go up
        } else if (down) {
            robot.sliderVertical.setPower(-0.6);
        } else {
            robot.sliderVertical.setPower(0);
        }
    }

    public void flipBox(boolean up, boolean down) {
        if (up) {
            robot.flippyBox.setPosition(robot.flippyBox.getPosition() - .081);   //assuming this (Y) is up .49
        } else if (down) {
            robot.flippyBox.setPosition(robot.flippyBox.getPosition() + .081);
        }
    }

    public void duckSpin(boolean left, boolean right) {
        if (left) {
            robot.duckSpinner.setPower(1);
        } else if (right) {
            robot.duckSpinner.setPower(-1);
        } else {
            robot.duckSpinner.setPower(0);
        }

    }

    public void theAbyss() {
        if (!isMoving)
        {
            robot.fsd.setPower(0);
            robot.bpd.setPower(0);
            robot.fpd.setPower(0);
            robot.bsd.setPower(0);
        }
        isMoving = false;
    }

    public void slowMode(boolean setSlow, boolean setFast) {
        if (setSlow) {
            speed = .40;
            sleep(40);
            telemetry.addData("Slow", "");
        } else if (setFast) {
            speed = 1;
            sleep(40);
            telemetry.addData("Fast", "");
        }
        telemetry.update();
    }

    public void drive(double stickX, double stickY) { //omni strafe ||Testing||
        double angle = Math.atan2(stickY, stickX);
        double magnitude = Math.sqrt(Math.pow(stickY, 2) + Math.pow(stickX, 2));
        if (stickX > 0.2 || stickX < -0.2 || stickY < -0.2 || stickY > 0.2) {
            isMoving = true;
            robot.fsd.setPower((Math.sin(angle + Math.PI / 4)) * magnitude * speed);//cos maybe?
            robot.bpd.setPower((Math.sin(angle + Math.PI / 4)) * magnitude * speed);
            robot.fpd.setPower((Math.sin(angle - Math.PI / 4)) * magnitude * speed);
            robot.bsd.setPower((Math.sin(angle - Math.PI / 4)) * magnitude * speed);
        }
    }

    public void spin(double stickX) {

        if (stickX > 0.15 || stickX < -0.15) {    //clockwise
            isMoving = true;
            robot.fpd.setPower(-stickX * speed);
            robot.bpd.setPower(-stickX * speed);
            robot.fsd.setPower(stickX * speed);
            robot.bsd.setPower(stickX * speed);
        }
    }

    public void doLights() {

        if (robot.dist.getDistance(DistanceUnit.CM) < 10)
        {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
        else if (robot.fsr.getVoltage() > .9) {
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);//heavy
    } else if (robot.fsr.getVoltage() > .7) {
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    } else if (robot.fsr.getVoltage() > .2) {
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);//light
    }
        else if (robot.color.alpha() > 95)
        {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
        }
        else
        {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
    }

    public void testInfo() {
        //telemetry.addData("red",robot.color.red());
        //telemetry.addData("blue",robot.color.blue());
        //telemetry.addData("green",robot.color.green());
        telemetry.addData("alpha",robot.color.alpha());
        telemetry.addData("Voltage",robot.fsr.getVoltage());
        telemetry.addData("Distance (cm)",robot.dist.getDistance(DistanceUnit.CM));
        telemetry.addData("Box Position", robot.flippyBox.getPosition());
        telemetry.addData("Arm Position",robot.sliderVertical.getCurrentPosition());
        telemetry.addData("Speed",speed);
        telemetry.update();
    }

    public void drive2(double stickX, double stickY) { //4 directional drive
        if (stickX < 0.2 && stickX > -0.2 && stickY < -0.35 || stickY > 0.35) {  //forward & backward
            isMoving = true;
            robot.fpd.setPower(stickY * speed);
            robot.bpd.setPower(stickY * speed);
            robot.fsd.setPower(stickY * speed);
            robot.bsd.setPower(stickY * speed);
        } else if (stickX < -0.25 && (stickY > -0.2 && stickY < 0.2)) {  //left and right (might be backwards)
            isMoving = true;
            robot.fsd.setPower(stickX * speed);
            robot.bsd.setPower(-stickX * speed);
            robot.fpd.setPower(-stickX * speed);
            robot.bpd.setPower(stickX * speed);
        }

    }

    public void driveTank(double stickLY,double stickRY)
    {
        if (stickLY > 0.2 || stickLY < -0.2) {
            robot.fpd.setPower(stickLY * speed);
            robot.bpd.setPower(stickLY * speed);

        }else
        {
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
        }
        if (stickRY > 0.2 || stickRY < -0.2) {
            robot.fsd.setPower(stickRY * speed);
            robot.bsd.setPower(stickRY * speed);

        }
        else
        {
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
        }
    }

    public void driveFieldCentric(double stickX, double stickY) {

        if (stickY < -.25 || stickY > .25 || stickX < -.25 || stickX > .25) {
            isMoving = true;
            double angle = Math.atan2(stickY, stickX);
            double magnitude = Math.sqrt(Math.pow(stickY, 2) + Math.pow(stickX, 2));
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double currentHeading = angles.firstAngle;
            robot.fsd.setPower((Math.sin(currentHeading + angle + Math.PI / 4)) * magnitude * speed);
            robot.bpd.setPower((Math.sin(currentHeading + angle + Math.PI / 4)) * magnitude * speed);
            robot.fpd.setPower((Math.sin(currentHeading + angle - Math.PI / 4)) * magnitude * speed);
            robot.bsd.setPower((Math.sin(currentHeading + angle - Math.PI / 4)) * magnitude * speed);
        }
    }
    /*public void displayLocation(){
        //double x, y, t, t2 = 0;
        //ConcurrentHashMap<Double,Double> xa, ya, xv, yv, xp, yp = new ConcurrentHashMap<>();
        Acceleration acceleration = imu.getAcceleration(); // m/s/s
        x = acceleration.xAccel;
        y = acceleration.yAccel;
        t = runtime.milliseconds();
            xa.put(t, x);
            ya.put(t, y);
            xv.put(t, integrate(t2, t, xa));
            yv.put(t, integrate(t2, t, ya));
            xp.put(t, integrate(t2, t, xv));
            yp.put(t, integrate(t2, t, yv));
            t2 = t;
            telemetry.addData("x", xp.get(t));
            telemetry.addData("y", yp.get(t));
            //telemetry.update();


    }
    /*public double integrate(double a, double b, ConcurrentHashMap<Double,Double> f) {
        if (f.containsKey(a) && f.containsKey(b))
        {
            return (b-a)*(f.get(a)+f.get(b)*0.5);
        }
        return -1;
    }
    /* public void driveGood(boolean button) //drive over bump and have back wheel in between obstacles
    {
        if (button)
        {
            while(robot.color.alpha() < 90)
            {
                robot.fsd.setPower(1);
                robot.bsd.setPower(1);
                robot.fpd.setPower(1);
                robot.bpd.setPower(1);
            }
            encoderDriveNoWait(1,20,20,3);
            encoderDriveNoWait(.6,8,8,3);
        }
    }
     */
}

/**
 *
 * this turns to an angle and then once it's straight continues at that angle
 *
 *  public void setMotorDirDrive() {
 *         robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE);
 *         robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
 *         robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
 *         robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);
 *
 *     }
 * public void driveFieldCentric(double stickX, double stickY) //field centered drive
 *     {
 *         if (stickY < -.25 || stickY > .25 || stickX < -.25 || stickX > .25)
 *         {
 *             double angle = Math.atan2(stickY, stickX);
 *             double magnitude = Math.sqrt(Math.pow(stickY,2)+Math.pow(stickX,2));
 *             angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
 *             double currentHeading = angles.firstAngle;
 *
 *
 *             robot.fsd.setPower((Math.sin(-startHeading+currentHeading+angle+Math.PI/4))*magnitude);
 *             robot.bpd.setPower((Math.sin(-startHeading+currentHeading+angle-Math.PI/4))*magnitude);
 *             robot.fpd.setPower((Math.sin(-startHeading+currentHeading+angle-Math.PI/4))*magnitude);
 *             robot.bsd.setPower((Math.sin(-startHeading+currentHeading+angle+Math.PI/4))*magnitude);
 *
 *         }
 *         else{
 *             robot.fpd.setPower(0.0);
 *             robot.bpd.setPower(0.0);
 *             robot.fsd.setPower(0.0);
 *             robot.bsd.setPower(0.0);
 *
 *         }
 *     }
 */


