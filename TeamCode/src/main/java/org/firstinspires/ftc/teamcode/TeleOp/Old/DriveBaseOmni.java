package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.LewaHardware;


    public class DriveBaseOmni extends AutoBase{
    LewaHardware robot = new LewaHardware();
    int mode2 = 0; //imu mode
    double speed = .75;
    boolean slow = false;
    BNO055IMU imu;
    Orientation angles;



        @Override
        public void runOpMode() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            robot.init(hardwareMap);

            telemetry.addData("Status:", "Run time dudes");
            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        }

        public void slowMode(boolean slowToggle) {
            if (!slow && slowToggle) {
                speed = .3;
                sleep(100);
                slow = true;
                telemetry.addData("Slow","");
            }
            else if (slow && slowToggle) {
                speed = .75;
                sleep(100);
                slow = false;
                telemetry.addData("Fast","");
            }
        }

        public void drive(double stickY) {
            if (stickY > .15 || stickY < -.15) {
                robot.fpd.setPower(-stickY*speed);
                robot.bpd.setPower(stickY*speed);
                robot.fsd.setPower(-stickY*speed);
                robot.bsd.setPower(stickY*speed);
            } else {
                robot.fpd.setPower(0.0);
                robot.bpd.setPower(0.0);
                robot.fsd.setPower(0.0);
                robot.bsd.setPower(0.0);

            }
        }

        public void driveWithIMU(double stickY) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (mode2 == 0) {
                if (mode2 == 0 && (angles.firstAngle > (8 * Math.PI / 9) || angles.firstAngle < (-8 * Math.PI / 9))) {
                    mode2 = 1;
                } else {
                    if (stickY > .15 || stickY < -.15) {
                        robot.fpd.setPower(-stickY*speed);
                        robot.bpd.setPower(stickY*speed);
                        robot.fsd.setPower(-stickY*speed);
                        robot.bsd.setPower(stickY*speed);
                    } else {
                        robot.fpd.setPower(0.0);
                        robot.bpd.setPower(0.0);
                        robot.fsd.setPower(0.0);
                        robot.bsd.setPower(0.0);
                    }
                }
            } else {
                if (mode2 == 1 && (angles.firstAngle < (Math.PI / 9) || angles.firstAngle > (-Math.PI / 9))) {
                    mode2 = 0;
                }
                if (stickY > .15 || stickY < -.15) {
                    robot.fpd.setPower(stickY*speed);
                    robot.bpd.setPower(-stickY*speed);
                    robot.fsd.setPower(stickY*speed);
                    robot.bsd.setPower(-stickY*speed);
                } else {
                    robot.fpd.setPower(0.0);
                    robot.bpd.setPower(0.0);
                    robot.fsd.setPower(0.0);
                    robot.bsd.setPower(0.0);

                }
            }

        }

        public void spin(double stickX) {
            if (stickX > 0.15 || stickX < -0.15) {    //clockwise
                robot.fpd.setPower(stickX);
                robot.bpd.setPower(-stickX);
                robot.fsd.setPower(-stickX);
                robot.bsd.setPower(stickX);
            }
        }

        public void intake(double in, double out) {
            if (in > 0) {
                robot.intakeMotor.setPower(1);//intake in
            }
            else if (out > 0) {
                robot.intakeMotor.setPower(-.6);
            } else {
                robot.intakeMotor.setPower(0);
            }
        }

        public void armSlider(boolean up, boolean down) {
            if (up) {
                robot.sliderVertical.setPower(1);//arm is supposed to go up
            }
            else if (down) {
                robot.sliderVertical.setPower(-0.7);
            } else {
                robot.sliderVertical.setPower(0);
            }
        }

        public void flipBox(boolean up, boolean down) {
            if (up) {
                robot.flippyBox.setPosition(robot.flippyBox.getPosition() - .01);
            } else if (down) {
                robot.flippyBox.setPosition(robot.flippyBox.getPosition() + .01);
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

        public void driveTank(double Lstick, double Rstick) {
            if (Lstick > .15 || Lstick < -.15) {
                robot.bpd.setPower(-Lstick*speed);
                robot.fpd.setPower(Lstick*speed);
            } else {
                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
            }
            if (Rstick > .15 || Rstick < -.15) {
                robot.fsd.setPower(-Rstick*speed);
                robot.fsd.setPower(Rstick*speed);
            } else {
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);
            }

        }

        public void testDistance() {
            telemetry.addData("Distance"," " + robot.dist.getDistance(DistanceUnit.CM) + "cm");
            telemetry.update();
        }

        //public void lightStuff() {
        /*double d = robot.dist.getDistance(DistanceUnit.CM);
        if (d < 18)
        {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if (d < 22)
        {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        else
        {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
    } */


}
