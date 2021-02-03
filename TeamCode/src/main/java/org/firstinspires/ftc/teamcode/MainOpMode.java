package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name="MainOpMode")
public class MainOpMode extends LinearOpMode {
    // Data field.
    private VuforiaCurrentGame vuforiaUltimateGoal;
    private TfodCurrentGame tfodUltimateGoal;
    private SampleMecanumDrive drive;
    private Trajectory trajectory;

    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;

    private Servo hopperangle;
    private Servo indexer;
    private Servo elbow;
    private CRServo jaw;

    Recognition recognition;
    double WobbleGoalZone;
    double ShooterPowerSettingLow;
    double ShooterPowerSettingHigh;
    double IndexerUpPosition;
    double IndexerDownPosition;
    double ElbowForwardPosition;
    double ElbowBackward;

    double ShootingTime;

    /**
     * Operation mode method.
     */
    @Override
    public void runOpMode() {
        // Initialize variables.
        List<Recognition> recognitions;
        double index;

        // Initialize necessary engines and devices.
        vuforiaUltimateGoal = new VuforiaCurrentGame();
        drive               = new SampleMecanumDrive(hardwareMap);
        tfodUltimateGoal    = new TfodCurrentGame();
        hopperangle         = hardwareMap.get(Servo.class, "hopperangle");
        shooter1            = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2            = hardwareMap.get(DcMotor.class, "shooter2");
        fl                  = hardwareMap.get(DcMotor.class, "fl");
        bl                  = hardwareMap.get(DcMotor.class, "bl");
        fr                  = hardwareMap.get(DcMotor.class, "fr");
        br                  = hardwareMap.get(DcMotor.class, "br");
        indexer             = hardwareMap.get(Servo.class, "indexer");
        elbow               = hardwareMap.get(Servo.class, "elbow");
        jaw                 = hardwareMap.get(CRServo.class, "jaw");

        // Initialize TensorFlow engine.
        tfodUltimateGoal.initialize(
                vuforiaUltimateGoal,
                0.7F, // Set minimum confidence threshold to 0.7.
                true,
                true
        );

        // Activate and configure TensorFlow recognition engine.
        tfodUltimateGoal.activate();
        tfodUltimateGoal.setZoom(2.5, 16 / 9.0);

        // Update telemetry.
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        // Configure all devices and variables before operation.
        hopperangle.setPosition(0.29);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterPowerSettingHigh = 0.9;
        ShooterPowerSettingLow  = 0.6;
        IndexerUpPosition       = 0.2;
        IndexerDownPosition     = 0.8;
        ElbowForwardPosition    = 0.7;
        ElbowBackward           = 0;
        ShootingTime            = 2;

        // Wait for start command from Driver Station.
        waitForStart();

        // Run operation.
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get recognitions from TensorFlow engine.
                recognitions = tfodUltimateGoal.getRecognitions();

                // Inform user if there are no recognitions.
                if (recognitions.size() == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("TargetZone", "A");
                    WobbleGoalZone = 1;
                }

                // Otherwise, display info for each recognition.
                else {
                    // Initialize index variable.
                    index = 0;

                    // Iterate through recognitions.
                    for (Recognition recognition : recognitions) {
                        // Define recognition.
                        this.recognition = recognition;

                        // Display info regarding recognition.
                        displayInfo(index);

                        // Increment index.
                        index++;
                    }

                    // Update telemetry.
                    telemetry.update();

                    // Determine trajectory.
                    if (WobbleGoalZone == 2) {
                        middleZone();
                    } else if (WobbleGoalZone == 3) {
                        farZone();
                    } else {
                        closeZone();
                    }
                }
            }
        }
    }

    /**
     * Goes to closest square if there are 0 rings.
     */
    private void closeZone() {
        // Build trajectory.
        trajectory = drive.trajectoryBuilder(new Pose2d())
                // Spline to powershot position.
                .splineTo(new Vector2d(-62, -50), Math.toRadians(30))

                // Ready shooter motors.
                .addTemporalMarker(1, () -> {
                    shooter1.setPower(ShooterPowerSettingLow);
                    shooter2.setPower(ShooterPowerSettingLow);
                })

                // Shoot 3 powershots.
                .addDisplacementMarker(() -> {
                    // Set indexer positions.
                    for (int i = 0; i < 3; i++) {
                        // TODO Test how long servo takes at the Makerspace.
                        indexer.setPosition(IndexerUpPosition);
                        indexer.setPosition(IndexerDownPosition);
                        drive.turn(Math.toRadians(5));
                    }

                    // Turn off shooter motors.
                    shooter1.setPower(0);
                    shooter2.setPower(0);
                })

                // Spline to close zone.
                .splineTo(new Vector2d(51, -60), Math.toRadians(0))

                // Drop the wobble goal.
                .addDisplacementMarker(() -> elbow.setPosition(ElbowForwardPosition))
                .addTemporalMarker(4.7 + ShootingTime, () -> jaw.setPower(-1))
                .addTemporalMarker(5.2 + ShootingTime, () -> jaw.setPower(0))

                // Spline to second wobble goal.
                .splineTo(new Vector2d(-40, -25.5), Math.toRadians(0))

                // Pick up the second wobble goal.
                .addDisplacementMarker(() -> elbow.setPosition(ElbowForwardPosition))
                .addTemporalMarker(9.8 + ShootingTime, () -> jaw.setPower(1))
                .addTemporalMarker(10.3 + ShootingTime, () -> jaw.setPower(0))

                // Return trajectory.
                .build();

                // TODO Close elbow before shooting.
                // TODO Open elbow before parking.
    }

    /**
     * Goes to middle square if there is 1 ring.
     */
    private void middleZone() {

    }

    /**
     * Goes to farthest square if there are 4 rings.
     */
    private void farZone() {

    }

    /**
     * Shoot a ring from the robot.
     private void Shoot() {
     indexer.setPosition(IndexerUpPosition);
     sleep(1000);
     indexer.setPosition(IndexerDownPosition);
     sleep(1000);
     }
     */

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(double i) {
        // Display label info.
        telemetry.addData("label " + i, recognition.getLabel());

        // Display upper corner info.
        telemetry.addData("Left, Top" + i,
                recognition.getLeft() + recognition.getTop());

        // Display lower corner info.
        telemetry.addData("Right, Bottom" + i,
                recognition.getRight() + recognition.getBottom());

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
