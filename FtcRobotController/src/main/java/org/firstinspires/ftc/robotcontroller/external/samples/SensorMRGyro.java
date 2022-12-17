/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/*
 * This is an example LinearOpMode that shows how to use the Modern Robotics Gyro.
 *
 * The op mode assumes that the gyro sensor is attached to a Device Interface Module
 * I2C channel and is configured with a name of "gyro".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
*/
@TeleOp(name = "Sensor: MR Gyro", group = "Sensor")
@Disabled
public class SensorMRGyro extends LinearOpMode {

  /** In this sample, for illustration purposes we use two interfaces on the one gyro object.
   * That's likely atypical: you'll probably use one or the other in any given situation,
   * depending on what you're trying to do. {@link IntegratingGyroscope} (and it's base interface,
   * {@link Gyroscope}) are common interfaces supported by possibly several different gyro
   * implementations. {@link ModernRoboticsI2cGyro}, by contrast, provides functionality that
   * is unique to the Modern Robotics gyro sensor.
   */
  IntegratingGyroscope gyro;
  ModernRoboticsI2cGyro modernRoboticsI2cGyro;

  // A timer helps provide feedback while calibration is taking place
  ElapsedTime timer = new ElapsedTime();

  @Override
  public void runOpMode() {

    boolean lastResetState = false;
    boolean curResetState  = false;

    // Get a reference to a Modern Robotics gyro object. We use several interfaces
    // on this object to illustrate which interfaces support which functionality.
    modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
    // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
    // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
    // A similar approach will work for the Gyroscope interface, if that's all you need.

    // Start calibrating the gyro. This takes a few seconds and is worth performing
    // during the initialization phase at the start of each OpMode.
    telemetry.log().add("Gyro Calibrating. Do Not Move!");
    modernRoboticsI2cGyro.calibrate();

    // Wait until the gyro calibration is complete
    timer.reset();
    while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
      telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
      telemetry.update();
      sleep(50);
    }

    telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
    telemetry.clear(); telemetry.update();

    // Wait for the start button to be pressed
    waitForStart();
    telemetry.log().clear();
    telemetry.log().add("Press A & B to reset heading");

    // Loop until we're asked to stop
    while (opModeIsActive())  {

      // If the A and B buttons are pressed just now, reset Z heading.
      curResetState = (gamepad1.a && gamepad1.b);
      if (curResetState && !lastResetState) {
        modernRoboticsI2cGyro.resetZAxisIntegrator();
      }
      lastResetState = curResetState;

      // The raw() methods report the angular rate of change about each of the
      // three axes directly as reported by the underlying sensor IC.
      int rawX = modernRoboticsI2cGyro.rawX();
      int rawY = modernRoboticsI2cGyro.rawY();
      int rawZ = modernRoboticsI2cGyro.rawZ();
      int heading = modernRoboticsI2cGyro.getHeading();
      int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

      // Read dimensionalized data from the gyro. This gyro can report angular velocities
      // about all three axes. Additionally, it internally integrates the Z axis to
      // be able to report an absolute angular Z orientation.
      AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
      float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

      // Read administrative information from the gyro
      int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
      int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

      telemetry.addLine()
        .addData("dx", formatRate(rates.xRotationRate))
        .addData("dy", formatRate(rates.yRotationRate))
        .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
      telemetry.addData("angle", "%s deg", formatFloat(zAngle));
      telemetry.addData("heading", "%3d deg", heading);
      telemetry.addData("integrated Z", "%3d", integratedZ);
      telemetry.addLine()
        .addData("rawX", formatRaw(rawX))
        .addData("rawY", formatRaw(rawY))
        .addData("rawZ", formatRaw(rawZ));
      telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
      telemetry.update();
    }
  }

  String formatRaw(int rawValue) {
    return String.format("%d", rawValue);
  }

  String formatRate(float rate) {
    return String.format("%.3f", rate);
  }

  String formatFloat(float rate) {
    return String.format("%.3f", rate);
  }

    /**
     * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
     * determine which image is being presented to the robot.
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
     *
     * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
     * is explained below.
     */
    @Autonomous(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")

    public static class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {

        /*
         * Specify the source for the Tensor Flow Model.
         * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
         * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
         * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
         * Here we assume it's an Asset.    Also see method initTfod() below .
         */
        /*public DistanceSensor frontDistance, leftDistance, backDistance, rightDistance;*/
        int redValue, blueValue, greenValue;
        //control hub: 0 is front right, 1 is front left, 2 is back left, 3 is back right
        //expansion hub: arm = 0, arm2 is 1
        public DcMotor frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel, arm, arm2;
        public CRServo claw, claw2;

        public DistanceSensor backDistanceSensor;
        //the variables that keep track of movement
        double armSpeed, turnSpeed;
        //armExtension Value

        //list that stores the power of the wheels, [0] is front right, [1] is front left, [2] is back left, [3] is back right
        double[] wheels = new double[4];

        private static final String TFOD_MODEL_ASSET = "model_unquant.tflite";
        // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


        private static final String[] LABELS = {
                "Three",
                "One",
                "Two"
        };

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        private static final String VUFORIA_KEY =
                "AY+LAO//////AAABmXqpfGFoHkhWrS1QLc/3RBMitc3sWaySydW66iqgCgULFQBcw/vHIj1o9YeqLzsHNYgoJ4bigISHSEJ0aqFhiZ1r+rRJ0HhFgL4V88oT/6FlJFzRDwhtVX+72HEoEIZgiH2vZD5i5mQp11U9rz+oE/07CwUzmABaW9i4gI50lPJhve1K6xRd4ydgjVhdiJ/Ayz6X8mK+LBnX10KxS0cCEtbCi2texH/X9W+iMXoXk9bzfnKi8X0xJxOAO9F5R2Ja9wBvEZrQQFhf/e4wegFY3fTYqsKtRTaENLYtQFuPqelbRuFRR2qcIZ9Q677IXEM+Ydagzu1bj2lBc6ueZf5SYn0iaPOIWRp4ilLNlRj5wyZ4";

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        private VuforiaLocalizer vuforia;

        /**
         * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
         * Detection engine.
         */
        private TFObjectDetector tfod;


        public void runOpMode() {
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();
            initTfod();


            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {

                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can increase the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 16/9).
                tfod.setZoom(1.0, 16.0/9.0);
            }

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.addData("your mom", " ");
            telemetry.update();
            config();
            waitForStart();

            if (opModeIsActive()) {

                while (opModeIsActive()) {

                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                        if (updatedRecognitions != null) {
                            telemetry.addData("# Objects Detected", updatedRecognitions.size());
                            telemetry.addData("your mom1", " ");
                            telemetry.update();
                            // step through the list of recognitions and display image position/size information for each one
                            // Note: "Image number" refers to the randomized image orientation/number

                            for (Recognition recognition : updatedRecognitions) {
                                double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                                double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                                double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                                double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                                telemetry.addData(""," ");
                                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                                telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                                telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);


                            }

                            sleep(50);
                            arm("UP", 600);
                            //moves forward
                            move(0.5, 0.5, 0.5, 0.5, 550);
                            //turns 180
                            move(-.5, -.5, 0.5, 0.5, 2400);
                            rest(1000);
                            //moves forward until it detects the cup
                            double backDistance = backDistanceSensor.getDistance(DistanceUnit.INCH);
                            //while it stops at 4 inches, the distance is actually a lot shorter because the motors need time to stop moving
                            while (backDistance > 5) {
                                backDistance = backDistanceSensor.getDistance(DistanceUnit.INCH);
                                continuousMove(-0.5, -0.5, -0.5, -0.5);
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }

        /**
         * Initialize the Vuforia localization engine.
         */
        private void initVuforia() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.75f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 300;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

            // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
            // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
           //tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        }
        public void config(){

            //initializes the variables
          /*frontDistance = hardwareMap.get(DistanceSensor.class, "distance front");
          leftDistance = hardwareMap.get(DistanceSensor.class, "distance left");
          backDistance = hardwareMap.get(DistanceSensor.class, "distance back");
          rightDistance = hardwareMap.get(DistanceSensor.class, "distance right");
           */
            frontLeftWheel = hardwareMap.get(DcMotor.class, "front left");
            frontRightWheel = hardwareMap.get(DcMotor.class, "front right");
            backLeftWheel = hardwareMap.get(DcMotor.class, "back left");
            backRightWheel = hardwareMap.get(DcMotor.class, "back right");
            //left sensor is 0
            //right sensor is 1
            //back sensor is 2
            arm = hardwareMap.get(DcMotor.class, "arm motor");
            arm2 = hardwareMap.get(DcMotor.class, "arm motor 2");
            claw = hardwareMap.get(CRServo.class, "claw");
            claw2 = hardwareMap.get(CRServo.class, "claw 2");

            backDistanceSensor = hardwareMap.get(DistanceSensor.class, "back distance sensor");
            //set direction for motors
            //Old code: left is forward and right is backward
            //New code: right is forward and left is backwards
            frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
            frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
            backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
            backRightWheel.setDirection(DcMotor.Direction.REVERSE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        }
        public void move(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower, int duration) {
            frontLeftWheel.setPower(frontLeftPower);
            backLeftWheel.setPower(backLeftPower);
            frontRightWheel.setPower(frontRightPower);
            backRightWheel.setPower(backRightPower);
            sleep(duration);
        }
        public void continuousMove(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
            frontLeftWheel.setPower(frontLeftPower);
            backLeftWheel.setPower(backLeftPower);
            frontRightWheel.setPower(frontRightPower);
            backRightWheel.setPower(backRightPower);
        }
        public void arm(String direction, int duration) {
            if (direction.equals("UP")) {
                arm.setPower(0.75);
                arm2.setPower(-0.75);
                sleep(duration);
            }
            else if (direction.equals("DOWN")) {
                arm.setPower(-.5);
                arm2.setPower(0.5);
                sleep(duration);
            }
            arm.setPower(0);
            arm2.setPower(0);
        }
        public void up(int duration){

        }
        public void down(int duration){
            arm.setPower(-1.0);
            arm2.setPower(1.0);
            sleep(duration);
        }
        public void open(int duration){
            claw.setPower(-1);
            claw2.setPower(1);
            sleep(duration);
        }
        public void close(int duration){
            claw.setPower(1);
            claw2.setPower(-1);
            sleep(duration);
        }
        public void rest(int duration) {
            frontLeftWheel.setPower(0);
            backLeftWheel.setPower(0);
            frontRightWheel.setPower(0);
            backRightWheel.setPower(0);
            sleep(duration);
        }
    }
}
