package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private Servo jaw;

    Recognition recognition;
    double wobbleGoalZone;
    double shooterPowerSettingLow;
    double shooterPowerSettingHigh;
    double indexerUpPosition;
    double indexerDownPosition;
    double elbowForwardPosition;
    double elbowBackwardPosition;
    double jawOpenPosition;
    double jawClosePosition;

    double shooterPrepTime1;
    double shooterPrepTime2;

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
        jaw                 = hardwareMap.get(Servo.class, "jaw");

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
        shooterPowerSettingHigh = 0.9;
        shooterPowerSettingLow  = 0.6;
        indexerUpPosition       = 0.2;
        indexerDownPosition     = 0.8;
        elbowForwardPosition    = 0.7;
        elbowBackwardPosition   = 0;
        jawOpenPosition         = 0; // TODO set position for jaw, also needs tele-op.
        jawClosePosition        = 0;
        shooterPrepTime1 = 1; // TODO Test with robot, also estimate.
        shooterPrepTime2 = 0;

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
                    wobbleGoalZone = 1;
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
                    if (wobbleGoalZone == 2) {
                        middleZone();
                    } else if (wobbleGoalZone == 3) {
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
        trajectory = drive.trajectoryBuilder(new Pose2d(-62, -50)) // 1) Start.
                // 2) Spline to powershot position.
                .splineTo(new Vector2d(-1, -50), Math.toRadians(30))

                // Ready shooter motors.
                .addTemporalMarker(shooterPrepTime1, this::readyShooters)

                // Shoot 3 powershots.
                .addDisplacementMarker(() -> {
                    // Set hopper indexer positions.
                    for (int i = 0; i < 3; i++) {
                        // Shoot a ring.
                        shoot();

                        // Turn 5° counterclockwise.
                        drive.turn(Math.toRadians(5));
                    }

                    // Turn off shooter motors.
                    shooter1.setPower(0);
                    shooter2.setPower(0);
                })

                // 3) Spline to 1st drop-off point at far zone.
                .splineToSplineHeading(
                        new Pose2d(41, -53, Math.toRadians(0)), // Face north.
                        Math.toRadians(90) // Tangent at 30°.
                )

                // Drop the first wobble goal.
                .addDisplacementMarker(this::dropWobbleGoal)

                // 4) Spline to second wobble goal.
                .splineToSplineHeading(
                        new Pose2d(-40, -25.5, Math.toRadians(270)), // Face east.
                        Math.toRadians(270) // Tangent at 270°.
                )

                // Pick up the second wobble goal.
                .addDisplacementMarker(this::obtainWobbleGoal)

                // 5) Spline to starter stack.
                .splineToSplineHeading(
                        new Pose2d(-28.5, -36.5, Math.toRadians(0)),  // Face north.
                        Math.toRadians(0) // Tangent at 0°.
                )

                // TODO Run intake mechanism to collect 3 rings.



                // 6) Spline to shooting point.
                .splineTo(new Vector2d(-12, -36.5), Math.toRadians(0))

                // Ready shooter motors.
                .addTemporalMarker(1, this::readyShooters)

                // Shoot 3 rings into high goal.
                .addDisplacementMarker(() -> {
                    for (int i = 0; i < 3; i++)
                        shoot();
                })

                // Open elbow before parking.
                .addDisplacementMarker(() -> elbow.setPosition(elbowForwardPosition))

                // Drop the second wobble goal.
                .addDisplacementMarker(this::obtainWobbleGoal)

                // 7) Spline to 2nd drop-off point at far zone.
                .splineTo(new Vector2d(59, -35), Math.toRadians(270))

                // 8) Spline to parking point.
                .splineTo(new Vector2d(11, -30), Math.toRadians(90))

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
     */
    private void shoot() {
        // TODO Test how long servo takes at the Makerspace.
        indexer.setPosition(indexerUpPosition);
        indexer.setPosition(indexerDownPosition);
    }

    /**
     * Ready shooter motors.
     */
    private void readyShooters() {
        shooter1.setPower(shooterPowerSettingLow);
        shooter2.setPower(shooterPowerSettingLow);
    }

    /**
     * Drop wobble goal.
     */
    private void dropWobbleGoal() {
        elbow.setPosition(elbowForwardPosition);
        jaw.setPosition(jawOpenPosition);
    }

    private void obtainWobbleGoal() {
        elbow.setPosition(elbowForwardPosition);
        jaw.setPosition(jawClosePosition);
    }

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
            wobbleGoalZone = 2;
        } else if (recognition.getLabel().equals("Quad")) {
            telemetry.addData("TargetZone", "C");
            wobbleGoalZone = 3;
        } else {
            telemetry.addData("TargetZone", "UNKNOWN");
        }
    }
}
