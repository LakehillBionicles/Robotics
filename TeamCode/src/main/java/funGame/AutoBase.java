package funGame;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;


import java.util.List;

public class AutoBase extends LinearOpMode {

    botHardware robot = new botHardware();


    public final String VUFORIA_KEY =
            "AVK0fyL/////AAABmQssSk9mVk96kBdzR7SOTK+JU/ZuMv8QpQVI57/d6DD/7rre5EwfXruGh0zwQ89E17WeE8jAlYaJl1w/00wkfEZvZQ1uyP2oTDhP6HrP/Z5arStkHU17WDYwjrQpncwUqkOB57SHsilJHJ2f9/pR13+5mAIJKO1vSB9EeIkbjJep/oBUO+pWN763R9VIFXTmbGbmL9KPqTstz/kVTd0K6/hQGRReFT5EWGqlcH+8E5X34F6v+AcvNYO5DIbNJat7/iZuaDdlYAokrQcl1ayNMyljJRK4sL/iRAtBoUEFiY4aZw2RN9D7SS/tQZCCJxrseXaJ1pu3IxGd6ld/BeKWJt88N6KMrZlmjjhX7MW5TKyY";

    public ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 580;
    static final double DRIVE_GEAR_REDUCTION = (1 / 1.5);     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /*** {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.*/
    public VuforiaLocalizer vuforia;


    /*** {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.*/
    public TFObjectDetector tfod;

    public AutoBase() {
    }

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
        initTfod();

        telemetry.addData("Vision Ready", "_");
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

    }


public void encoderDrive (double speed, double leftInches, double rightInches, double timeoutS) {
    robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    int newLeftTarget = 0;
    int newRightTarget = 0;

    setMotorDir2();

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
                        (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2)))) {
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

public void sideways (double speed, double frontInches, double backInches, double timeoutS) {
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
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.setZoom(1, 16.0 / 9.0);
        tfod.activate();
    }

    /*public String detectDuck() {
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

     */


    /*public void SOMETHING WE WANT TO DO() {
        robot.duckSpinner.setPower(-1.0);
        sleep(4200);
        robot.duckSpinner.setPower(0);
    }*/



        }




