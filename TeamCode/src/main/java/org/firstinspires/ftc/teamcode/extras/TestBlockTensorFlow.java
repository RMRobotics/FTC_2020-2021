package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

@Autonomous(name = "TestBlockTensorFlow (Blocks to Java)")
public class TestBlockTensorFlow extends LinearOpMode {

    private VuforiaCurrentGame vuforiaUltimateGoal;
    private TfodCurrentGame tfodUltimateGoal;
    private Servo hopperangle;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;
    private Servo indexer;
    private Servo elbow;
    private CRServo jaw;

    Recognition recognition;
    double WobbleGoalZone;
    double ShooterPowerSetting;
    double IndexerUpPosition;
    double IndexerDownPosition;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        double index;

        vuforiaUltimateGoal = new VuforiaCurrentGame();
        tfodUltimateGoal = new TfodCurrentGame();
        hopperangle = hardwareMap.get(Servo.class, "hopperangle");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");
        indexer = hardwareMap.get(Servo.class, "indexer");
        elbow = hardwareMap.get(Servo.class, "elbow");
        jaw = hardwareMap.get(CRServo.class, "jaw");

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaUltimateGoal.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Frontcam"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                1, // dx
                5, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodUltimateGoal.activate();
        // Enable following block to zoom in on target.
        tfodUltimateGoal.setZoom(2.5, 16 / 9);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        hopperangle.setPosition(0.29);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterPowerSetting = 0.9;
        IndexerUpPosition = 0.2;
        IndexerDownPosition = 0.8;
        // Wait for start command from Driver Station.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            // Get a list of recognitions from TFOD.
            recognitions = tfodUltimateGoal.getRecognitions();
            // If list is empty, inform the user. Otherwise, go
            // through list and display info for each recognition.
            if (recognitions.size() == 0) {
                telemetry.addData("TFOD", "No items detected.");
                telemetry.addData("TargetZone", "A");
                WobbleGoalZone = 1;
            } else {
                index = 0;
                // Iterate through list and call a function to
                // display info for each recognized object.
                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                    // Display info.
                    displayInfo(index);
                    // Increment index.
                    index = index + 1;
                }
            }
            telemetry.update();
            if (WobbleGoalZone == 2) {
                MiddleZone();
            } else if (WobbleGoalZone == 3) {
                FarZone();
            } else {
                CloseZone();
            }
            while (opModeIsActive()) {
                // Put loop blocks here.
            }
        }
        // Deactivate TFOD.
        tfodUltimateGoal.deactivate();

        vuforiaUltimateGoal.close();
        tfodUltimateGoal.close();
    }

    /**
     * Describe this function...
     */
    private void CloseZone() {
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(1500);
        fl.setPower(-0.5);
        bl.setPower(-0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(25);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        shooter1.setPower(ShooterPowerSetting);
        shooter2.setPower(ShooterPowerSetting);
        sleep(2000);
        Shoot();
        Shoot();
        Shoot();
        indexer.setPosition(IndexerUpPosition);
        sleep(2000);
        shooter1.setPower(0);
        shooter2.setPower(0);
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(150);
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(500);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        elbow.setPosition(0.7);
        sleep(1000);
        jaw.setPower(-1);
        sleep(500);
        jaw.setPower(0);
        fl.setPower(-0.5);
        bl.setPower(-0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(250);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void MiddleZone() {
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(1500);
        fl.setPower(-0.5);
        bl.setPower(-0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(25);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        shooter1.setPower(ShooterPowerSetting);
        shooter2.setPower(ShooterPowerSetting);
        sleep(2000);
        Shoot();
        Shoot();
        Shoot();
        indexer.setPosition(IndexerUpPosition);
        sleep(2000);
        shooter1.setPower(0);
        shooter2.setPower(0);
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(50);
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(1250);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        elbow.setPosition(0.7);
        sleep(1000);
        jaw.setPower(-1);
        sleep(500);
        jaw.setPower(0);
        fl.setPower(-0.5);
        bl.setPower(-0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(500);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void FarZone() {
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(1500);
        fl.setPower(-0.5);
        bl.setPower(-0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(25);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        shooter1.setPower(ShooterPowerSetting);
        shooter2.setPower(ShooterPowerSetting);
        sleep(2000);
        Shoot();
        Shoot();
        Shoot();
        indexer.setPosition(IndexerUpPosition);
        sleep(2000);
        shooter1.setPower(0);
        shooter2.setPower(0);
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(125);
        fl.setPower(0.5);
        bl.setPower(0.5);
        fr.setPower(-0.5);
        br.setPower(-0.5);
        sleep(1500);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        elbow.setPosition(0.7);
        sleep(1000);
        jaw.setPower(-1);
        sleep(500);
        jaw.setPower(0);
        fl.setPower(-0.5);
        bl.setPower(-0.5);
        fr.setPower(0.5);
        br.setPower(0.5);
        sleep(750);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void Shoot() {
        indexer.setPosition(IndexerUpPosition);
        sleep(1000);
        indexer.setPosition(IndexerDownPosition);
        sleep(1000);
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(double i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display lower corner info.
        // Display Target Zone
        if (recognition.getLabel().equals("Single")) {
            telemetry.addData("TargetZone", "B");
            WobbleGoalZone = 2;
        } else if (recognition.getLabel().equals("Quad")) {
            telemetry.addData("TargetZone", "C");
            WobbleGoalZone = 3;
        } else {
            telemetry.addData("TargetZone", "UNKNOWN");
        }
    }
}
