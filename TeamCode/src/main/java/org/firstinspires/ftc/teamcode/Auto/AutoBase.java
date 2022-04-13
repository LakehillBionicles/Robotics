package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

public class AutoBase extends LinearOpMode {

    LewaHardware robot = new LewaHardware();
    BNO055IMU imu;
    Orientation angles;
    final double boxStart = .42;
    final double boxStop = .95;

    private static final String TFOD_MODEL_ASSET = "test_capstone.tflite";
    private static final String[] LABELS = {
            "cap"
    };
    public final String VUFORIA_KEY =
            "AVK0fyL/////AAABmQssSk9mVk96kBdzR7SOTK+JU/ZuMv8QpQVI57/d6DD/7rre5EwfXruGh0zwQ89E17WeE8jAlYaJl1w/00wkfEZvZQ1uyP2oTDhP6HrP/Z5arStkHU17WDYwjrQpncwUqkOB57SHsilJHJ2f9/pR13+5mAIJKO1vSB9EeIkbjJep/oBUO+pWN763R9VIFXTmbGbmL9KPqTstz/kVTd0K6/hQGRReFT5EWGqlcH+8E5X34F6v+AcvNYO5DIbNJat7/iZuaDdlYAokrQcl1ayNMyljJRK4sL/iRAtBoUEFiY4aZw2RN9D7SS/tQZCCJxrseXaJ1pu3IxGd6ld/BeKWJt88N6KMrZlmjjhX7MW5TKyY";

    public ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 560; ///22.9
    static final double DRIVE_GEAR_REDUCTION = (.39);     // This is < 1.0 if geared UP  .385 true
    static final double WHEEL_DIAMETER_INCHES = 3.4975;     // For figuring circumference    2.953!!!!!!!!!!!!!!!!
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double spedAdjust = .15;
    private int boundBE = 5;

    /*** {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.*/
    public VuforiaLocalizer vuforia;


    /*** {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.*/
    public TFObjectDetector tfod;

    public AutoBase() {
    }  //constructor

    @Override
    public void runOpMode() {
        startUp();
    }

    public void startUp() {
        robot.init(hardwareMap);

        setMotorDirDrive();

        robot.fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        initVuforia();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robot.init(hardwareMap);

           initTfod();

        /** Wait for the game to begin */
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        telemetry.addData("Vision is Ready", ")");
        telemetry.update();


    }
    public void startUpFast() {
        robot.init(hardwareMap);
        setMotorDirDrive();
        robot.fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        robot.flippyBox.scaleRange(.45,1);
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {  //forward back and turn with encoder, always do 3 less

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorDir2();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int) ((leftInches) * this.COUNTS_PER_INCH);
            newRightTarget = (int) ((rightInches) * this.COUNTS_PER_INCH);

            robot.fpd.setTargetPosition(newLeftTarget);
            robot.fsd.setTargetPosition(newRightTarget);
            robot.bpd.setTargetPosition(newLeftTarget);
            robot.bsd.setTargetPosition(newRightTarget);

            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int NT1 = (int) (newLeftTarget * 0.981); //fp
            int NT2 = (int) (newRightTarget * 0.981); //fp

            // reset the timeout time and start motion.
            runtime.reset();

            robot.fpd.setPower(-Math.abs(speed));
            robot.fsd.setPower(-Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));

            while (robot.fpd.isBusy() && robot.fsd.isBusy() && robot.bpd.isBusy() && robot.bsd.isBusy() &&
                    opModeIsActive() && (runtime.seconds() < timeoutS) &&
                            ((Math.abs(robot.fpd.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT2)) ||
                            (Math.abs(robot.bpd.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2))))
            {
                //empty loop body
            }

            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(40);
        }
    }

    public void encoderDriveNoWait(double speed, double leftInches, double rightInches, double timeoutS) {  //forward back and turn with encoder, always do 3 less

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorDir2();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int) ((leftInches) * this.COUNTS_PER_INCH);
            newRightTarget = (int) ((rightInches) * this.COUNTS_PER_INCH);

            robot.fpd.setTargetPosition(newLeftTarget);
            robot.fsd.setTargetPosition(newRightTarget);
            robot.bpd.setTargetPosition(newLeftTarget);
            robot.bsd.setTargetPosition(newRightTarget);

            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int NT1 = (int) (newLeftTarget * 0.981); //fp
            int NT2 = (int) (newRightTarget * 0.981); //fp

            // reset the timeout time and start motion.
            runtime.reset();

            robot.fpd.setPower(-Math.abs(speed));
            robot.fsd.setPower(-Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));

            while (robot.fpd.isBusy() && robot.fsd.isBusy() && robot.bpd.isBusy() && robot.bsd.isBusy() &&
                    opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    ((Math.abs(robot.fpd.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT2)) ||
                            (Math.abs(robot.bpd.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2))))
            {
                //empty loop body
            }

            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDriveOld(double speed, double leftInches, double rightInches, double timeoutS) {  //forward back and turn with encoder, always do 3 less

        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorDir2();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.fpd.getCurrentPosition() + (int) ((leftInches) * this.COUNTS_PER_INCH);
            newRightTarget = robot.fsd.getCurrentPosition() + (int) ((rightInches) * this.COUNTS_PER_INCH);

            robot.fpd.setTargetPosition(newLeftTarget);
            robot.fsd.setTargetPosition(newRightTarget);
            robot.bpd.setTargetPosition(newLeftTarget);
            robot.bsd.setTargetPosition(newRightTarget);

            int NT1 = (int) (newLeftTarget * 0.981); //fp
            int NT2 = (int) (newRightTarget * 0.981); //fp

            // Turn On RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            robot.fpd.setPower(-Math.abs(speed));
            robot.fsd.setPower(-Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                ((Math.abs(robot.fpd.getCurrentPosition()) < Math.abs(NT1)) || (Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT2)))) {
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //sleep(250);   // optional pause after each move
        }
    }
    public void encoderDrivePlus(double speed, double leftInches, double rightInches, double timeoutS) {

        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorDirDrive();


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.fpd.getCurrentPosition() + (int) ((leftInches) * this.COUNTS_PER_INCH);
            newRightTarget = robot.fsd.getCurrentPosition() + (int) ((rightInches) * this.COUNTS_PER_INCH);

            robot.fpd.setTargetPosition(newLeftTarget);
            robot.fsd.setTargetPosition(newRightTarget);
            robot.bpd.setTargetPosition(newLeftTarget);
            robot.bsd.setTargetPosition(newRightTarget);


            int NT1 = (int) (newLeftTarget * 0.981); //fp
            int NT2 = (int) (newRightTarget * 0.981); //fp


            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fpd.setPower(Math.abs(speed));
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(-Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.


            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                if (angles.firstAngle > .02) {
                    robot.fpd.setPower(Math.abs(speed) + .3);
                    while (angles.firstAngle > .02) {

                    }
                    robot.fpd.setPower(Math.abs(speed));
                } else if (angles.firstAngle < -.02) {
                    robot.fsd.setPower(Math.abs(speed) + .3);
                    while (angles.firstAngle < -.02) {

                    }
                    robot.fsd.setPower(Math.abs(speed));
                }
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            sleep(150);   // optional pause after each move
        }
    }

    public void sideways(double speed, double frontInches, double backInches, double timeoutS) {
        int newFrontTarget = 0;
        int newBackTarget = 0;

        setMotorDirStrafe();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newFrontTarget = robot.fsd.getCurrentPosition() + (int) (frontInches * this.COUNTS_PER_INCH);
            newBackTarget = robot.bsd.getCurrentPosition() + (int) (backInches * this.COUNTS_PER_INCH);

            robot.fpd.setTargetPosition(newFrontTarget);
            robot.fsd.setTargetPosition(newFrontTarget);
            robot.bpd.setTargetPosition(newBackTarget);
            robot.bsd.setTargetPosition(newBackTarget);


            int NT1 = (int) (newFrontTarget * 0.981); //fs
            int NT2 = (int) (newBackTarget * 0.981); //bs


            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fpd.setPower(Math.abs(speed));  //-
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));  //-
            robot.bpd.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    // ( front_star_wheel.isBusy() && front_port_wheel.isBusy()))
                    ((Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT1)) || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2)))) { // took this out for now || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2))

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontTarget, newBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.fsd.getCurrentPosition(),
                        robot.bsd.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            sleep(250);   // optional pause after each move
        }


    }

    public void correctForBack(double speed) {

        double sped = (speed - spedAdjust);  //adjusted speed

        if (robot.bpd.getCurrentPosition() > boundBE) { // too much port???

            while (robot.bpd.getCurrentPosition() > boundBE) {  //correct while its bad

                //decrease port side motor speed for adjustments, at spEd
                robot.bpd.setPower(sped);
                robot.fpd.setPower(sped);

            }

            //put port side motor speed back where it should be, at spEEd
            robot.bpd.setPower(speed);
            robot.fpd.setPower(speed);

        } else if (robot.bpd.getCurrentPosition() < -boundBE) {   //too much star???

            while (robot.bpd.getCurrentPosition() < -boundBE) {

                //decrease star side motor speed for adjustments, at spEd
                robot.bsd.setPower(sped);
                robot.fsd.setPower(sped);

            }

            //put star side motor speed back where it should be, at spEEd
            robot.bsd.setPower(sped);
            robot.fsd.setPower(sped);

        }
    }

    public void correctSideways(double speed) {

        double sped = (speed - spedAdjust);

        if (robot.bsd.getCurrentPosition() > boundBE) { // too much forward side

            while (robot.bsd.getCurrentPosition() > boundBE) {
                //decrease front side motor speed for adjustments, at spEd
                robot.fpd.setPower(sped);
                robot.fsd.setPower(sped);

            }

            //put front side motor speed back where it should be, at spEEd
            robot.fsd.setPower(speed);
            robot.fpd.setPower(speed);

        } else if (robot.bsd.getCurrentPosition() < -boundBE) {   //too much back side

            while (robot.bsd.getCurrentPosition() < -boundBE) {
                //decrease back side motor speed for adjustments, at spEd
                robot.bsd.setPower(sped);
                robot.bpd.setPower(sped);

            }

            //put back side motor speed back where it should be, at spEEd
            robot.bsd.setPower(sped);
            robot.bpd.setPower(sped);

        }
    }

    public void setMotorDirDrive() {
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bsd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void setMotorDir2() {
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bsd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    public void setMotorDirStrafe(){
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);
    }



    /**
     * public void initVuforia() {
     * <p>
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     * <p>
     * VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
     * <p>
     * parameters.vuforiaLicenseKey = VUFORIA_KEY;
     * parameters.cameraName = hardwareMap.get(WebcamName.class, "eyes");
     * //parameters.webcamCalibrationFiles("");
     * //parameters.camera.createCaptureSession();
     * //parameters.cameraMonitorFeedback();
     * <p>
     * //  Instantiate the Vuforia engine
     * vuforia = ClassFactory.getInstance().createVuforia(parameters);
     * <p>
     * // Loading trackables is not necessary for the TensorFlow Object Detection engine.
     * }
     */

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.setZoom(1, 16.0 / 9.0);
        tfod.activate();
    }

    public String detectDuck() {
        String position = "";
        int k = 0;
         do {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0) { telemetry.addData("Position","top");
                    position = "top";
                    }
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {

                        if (recognition.getLeft() > 400) {
                            telemetry.addData("Position","middle");
                            position = "middle";
                        } else if (recognition.getLeft() < 400) {
                            telemetry.addData("Position","bottom");
                            position = "bottom";
                        }

                        i++;
                    }

                    telemetry.update();
                    return position;
                }
            }
            k++;
        } while (opModeIsActive() && position.equals("s") && k < 1000);
        return position;

    }

    public void spinCarousel() {
        robot.duckSpinner.setPower(-1.0);
        sleep(4200);
        robot.duckSpinner.setPower(0);
    }

    public void spinCarouselRed() {
        robot.duckSpinner.setPower(1.0);
        sleep(4200);
        robot.duckSpinner.setPower(0);
    }

    public void liftFreight(String duckPosition) {
        double in = 0;
        if (duckPosition.equals("bottom")) {
            in = 155;

        } else if (duckPosition.equals("middle")) {
            in = 260;

        } else if (duckPosition.equals("top")) {
            in = 400;
        }
        sliderLift(.6,1,in);
        robot.flippyBox.setPosition(boxStop);
    }

    public void sliderLift(double speed, double timeoutS, double inches) { //backwards?
        robot.sliderVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget = 0;

        setMotorDirDrive();
        if (opModeIsActive()) {

            newTarget = (int) inches; //inches * this.COUNTS_PER_INCH?
            robot.sliderVertical.setTargetPosition(newTarget);
            robot.sliderVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int NT = (int) (newTarget * 0.981); //fp
            runtime.reset();
            robot.sliderVertical.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && robot.sliderVertical.isBusy() &&
                    ((Math.abs(robot.sliderVertical.getCurrentPosition()) < Math.abs(NT)))) {
            }
            robot.sliderVertical.setPower(0);
            robot.sliderVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
    public void resetArm()
    {
        robot.flippyBox.setPosition(boxStart);
        sleep(50);
        robot.sliderVertical.setTargetPosition(0);
        robot.sliderVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.sliderVertical.setPower(-.6);
        while (opModeIsActive() && ((Math.abs(robot.sliderVertical.getCurrentPosition()) > 10))) {}
        robot.sliderVertical.setPower(0);
        robot.sliderVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn2(double degreesRightNegLeftPos, double speed, Orientation A){ //right is positive left is negative
        if (degreesRightNegLeftPos > 0) //turn left
            {
                double x = A.firstAngle;
            while (A.firstAngle < x + degreesRightNegLeftPos * (Math.PI / 180)) {
                robot.fpd.setPower(-speed);
                robot.bpd.setPower(speed);
                robot.fsd.setPower(speed);
                robot.bsd.setPower(-speed);
                updateAngles();
            }
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
        }
        if (degreesRightNegLeftPos < 0) {
            double y = A.firstAngle;
            while (A.firstAngle > y + degreesRightNegLeftPos * (Math.PI / 180)) {
                robot.fpd.setPower(speed);
                robot.bpd.setPower(-speed);
                robot.fsd.setPower(-speed);
                robot.bsd.setPower(speed);
                updateAngles();
            }
        }
    }
    public void turn90Right() { encoderDrive(.8,-16,16,3); }
    public void turn90Left() { encoderDrive(.8,16,-16,3); }
    public void turn45Right() { encoderDrive(.9,-10,10,3); }
    public void turn45Left() { encoderDrive(.9,10,-10,3); }

    public void turn(double degrees, double speed) {//positive is left, doesn't really work
        double radians = degrees * (Math.PI / 180);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        if (radians > Math.PI) {
            radians = -Math.PI - (-radians) % Math.PI;
        }
        if (radians < -Math.PI) {
            radians = Math.PI - (-radians) % Math.PI;
        }
        //heading < end -> right
            //unless -180 to +180 -> left
        //heading > end -> left
            //unless +180 to -180 -> right

        if (angles.firstAngle < radians && angles.firstAngle < -Math.PI/2 && radians > Math.PI/2) //left
        {
            while (angles.firstAngle < radians - .02) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                robot.fpd.setPower(speed);
                robot.bpd.setPower(speed);
                robot.fsd.setPower(-speed);
                robot.bsd.setPower(-speed);
            }
        }
        else if (angles.firstAngle < radians)//right
        {
            while (angles.firstAngle < radians - .02) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                robot.fpd.setPower(-speed);
                robot.bpd.setPower(-speed);
                robot.fsd.setPower(speed);
                robot.bsd.setPower(speed);
            }
        }
        if (angles.firstAngle > radians && angles.firstAngle > Math.PI/2 && radians < -Math.PI/2) //right
        {
            while (angles.firstAngle < radians - .02) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                robot.fpd.setPower(-speed);
                robot.bpd.setPower(-speed);
                robot.fsd.setPower(speed);
                robot.bsd.setPower(speed);
            }
        }
        else //left
        {
            while (angles.firstAngle < radians - .02) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                robot.fpd.setPower(speed);
                robot.bpd.setPower(speed);
                robot.fsd.setPower(-speed);
                robot.bsd.setPower(-speed);
            }
        }
    }

    public void updateAngles() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); //NullPointerException!!??
    }

    public void colorSense() {
        while (robot.color.alpha() < 95 && runtime.seconds() < 4.5)
        {
            robot.fpd.setPower(-.38);
            robot.fsd.setPower(-.38);
            robot.bsd.setPower(-.38);
            robot.bpd.setPower(-.38);
        }
        robot.fpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
        robot.bpd.setPower(0);
        encoderDrive(.6,-3,-3,2);
    }
    public void colorSenseFar() {
        while (robot.color.alpha() < 95 && runtime.seconds() < 4.5)
        {
            robot.fpd.setPower(-.38);
            robot.fsd.setPower(-.38);
            robot.bsd.setPower(-.38);
            robot.bpd.setPower(-.38);
        }
        robot.fpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
        robot.bpd.setPower(0);
        encoderDrive(.6,-12,-12,2);
    }

    public void turn(double degrees) {//can't turn >= 180 degrees
        double radians = Math.PI / 180 * degrees;
        if (gamepad1.x) { //left turn 90 degrees
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            imu.initialize(parameters);
            //power wheels
        } else if (degrees == 180) {
            while (angles.firstAngle > -0.1) {
                {

                }
                //stop wheels
            }
        } else if (degrees == -180) {
            while (angles.firstAngle < 0.1) {
                {

                }
                //stop wheels
            }
        }
        else{

                while (angles.firstAngle < radians) {
                    {

                    }
                    //stop wheels
                }
            }
        }

        //drive forward until color = white; then stop; turn wheel klasdhglashd more times

                 /*public void testDistance() {
        robot.dist.getDistance(DistanceUnit.CM);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        if(robot.dist.getDistance(DistanceUnit.CM) <= 10) {
            speed = 0.5;
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        }
        else if(robot.dist.getDistance(DistanceUnit.CM) > 10) {
            speed = 1;
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }
                  */



    }


