package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
    private DcMotor intake;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;

    private Servo jaw;
    private Servo cameraServo;
    private Servo hopper;
    private Servo indexer;
    private Servo elbow;

    Recognition recognition;
    double wobbleGoalZone;

    double shooterPowerSettingLow;
    double shooterPowerSettingHigh;
    double intakePowerSetting;

    double indexerUpPosition;
    double indexerDownPosition;
    double elbowDownPosition;
    double elbowUpPosition;
    double hopperInputAngle;
    double hopperOutputAngle;
    double jawOpenPosition;
    double jawClosePosition;

    double intakeDistance;
    double powershotAngle;
    double openIntakePosition;

    double shootingDuration;
    double shooterPrepTime1;
    double shooterPrepTimeFarZone2;
    double shooterTimeMiddleZone2;

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
        shooter1            = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2            = hardwareMap.get(DcMotor.class, "shooter2");
        intake              = hardwareMap.get(DcMotor.class, "intake");
        fl                  = hardwareMap.get(DcMotor.class, "fl");
        bl                  = hardwareMap.get(DcMotor.class, "bl");
        fr                  = hardwareMap.get(DcMotor.class, "fr");
        br                  = hardwareMap.get(DcMotor.class, "br");
        jaw                 = hardwareMap.get(Servo.class, "jaw");
        cameraServo         = hardwareMap.get(Servo.class, "cameraServo");
        hopper              = hardwareMap.get(Servo.class, "hopper");
        indexer             = hardwareMap.get(Servo.class, "indexer");
        elbow               = hardwareMap.get(Servo.class, "elbow");

        // Configure all variables before operation.
        shooterPowerSettingHigh     = 0.9;
        shooterPowerSettingLow      = 0.6;
        intakePowerSetting          = 1;

        indexerUpPosition           = 0.3;      // TODO fix wth robot.
        indexerDownPosition         = 0;        // TODO fix wth robot.
        elbowDownPosition = 0.7;
        elbowUpPosition = 0.28;
        hopperInputAngle            = 0.1;      // TODO Hopper up position = 0.29
        hopperOutputAngle           = 0.29;     // TODO Hopper down position = 0.1
        jawOpenPosition             = 0.1;      // TODO set position for jaw, also needs tele-op.
        jawClosePosition            = 0.36;     // TODO set position for jaw, also needs tele-

        intakeDistance              = 0;        // TODO fix with robot, currently set to 0.
        powershotAngle              = 5;        // TODO test at Makerspace.
        openIntakePosition          = 0.11;

        shootingDuration            = 4;
        shooterPrepTime1            = 1;        // TODO Test with robot, also estimate.
        shooterPrepTimeFarZone2     = 10        // TODO Use road runner GUI.
                + shootingDuration;
        shooterTimeMiddleZone2      = 8
                + shootingDuration;

        // Configure all devices before operation.
        hopper.setPosition(0.29);
        cameraServo.setPosition(0.17);
        jaw.setPosition(jawClosePosition);
        elbow.setPosition(elbowDownPosition);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize vuforia engine.
        vuforiaUltimateGoal.initialize(
                "",
                hardwareMap.get(WebcamName.class, "Frontcam"), // cameraName
                "",
                false,
                false,
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                1, // dx
                5, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true
        );

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

        // Wait for start command from Driver Station.
        waitForStart();

        // Run operation.
        if (opModeIsActive()) {
            // TODO Turn camera servo 30° to the right.

            // Get recognitions from TensorFlow engine.
            recognitions = tfodUltimateGoal.getRecognitions();

            // If there are no recognitions.
            if (recognitions.size() == 0) {
                telemetry.addData("TFOD", "No items detected.");
                telemetry.addData("TargetZone", "A");
                wobbleGoalZone = 1;
            }

            // If there are recognitions.
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
            }

            // Determine trajectory.
            if (wobbleGoalZone == 2) {
                telemetry.addData("TargetZone", "B");
                middleZone();
            } else if (wobbleGoalZone == 3) {
                telemetry.addData("TargetZone", "A");
                farZone();
            } else if (wobbleGoalZone == 1){
                telemetry.addData("TargetZone", "C");
                closeZone();
            }

            // Stop if requested.
            if (isStopRequested()) return;

            // Follow trajectory.
            drive.followTrajectory(trajectory);
        }

        // Close tensorflow engine.
        tfodUltimateGoal.close();
    }

    /**
     * Goes to farthest square if there are 4 rings.
     */
    private void closeZone() {
        // Build trajectory.
        trajectory = drive.trajectoryBuilder(new Pose2d(-62, -50)) // 1) Start.
                // 2) Spline to powershot position.
                .splineTo(new Vector2d(-1, -50), Math.toRadians(30))

                // Ready shooter motors.
                //.addTemporalMarker(shooterPrepTime1, this::activateShooters)
                .addDisplacementMarker(() -> {
                    /*telemetry.addData("Debug", "asdf");
                    telemetry.update();*/
                })

                // Shoot 3 powershots.
                .addDisplacementMarker(() -> {
                    /*shootPowershots();
                    telemetry.addData("Debug", "hjkl");
                    telemetry.update();
                    deactivateShooters();*/
                })

                // 3) Spline to 1st drop-off point at close zone at heading of 270°.
                .splineToSplineHeading(
                        new Pose2d(11, -40, Math.toRadians(270)),
                        Math.toRadians(0) // Tangent
                )

                // Drop the first wobble goal.
                .addDisplacementMarker(this::dropWobbleGoal)

                // 4) Spline to second wobble goal.
                .splineToLinearHeading(
                        new Pose2d(-88, -5.5, Math.toRadians(270)),
                        Math.toRadians(270)
                )

                // Pick up the second wobble goal.
                .addDisplacementMarker(this::obtainWobbleGoal)

                /*// 5) Spline to 2nd drop-off point at close zone.
                .splineTo(new Vector2d(-10, -60), Math.toRadians(0))

                // Drop the second wobble goal.
                .addDisplacementMarker(this::dropWobbleGoal)

                // 6) Spline to parking point.
                .splineTo(new Vector2d(11, -30), Math.toRadians(0))

                // TODO Remove after testing, will mess up auto path.
                .splineToSplineHeading(
                        new Pose2d(-62, -50, Math.toRadians(0)),
                        Math.toRadians(180)
                )*/

                // Return trajectory.
                .build();
    }

    /**
     * Goes to middle square if there is 1 ring.
     */
    private void middleZone() {
        // Build trajectory.
        trajectory = drive.trajectoryBuilder(new Pose2d(-62, -50)) // 1) Start.
                // 2) Spline to powershot position.
                .splineTo(new Vector2d(-1, -50), Math.toRadians(30))

                // Ready shooter motors.
                .addTemporalMarker(shooterPrepTime1, this::activateShooters)

                // Shoot 3 powershots.
                .addDisplacementMarker(() -> {
                    shootPowershots();
                    deactivateShooters();
                })

                // 3) Spline to 1st drop-off point at middle zone.
                .splineToSplineHeading(
                        new Pose2d(15, -33, Math.toRadians(0)),
                        Math.toRadians(90)
                )

                // Drop the first wobble goal.
                .addDisplacementMarker(this::dropWobbleGoal)

                // 4) Spline to second wobble goal.
                .splineToLinearHeading(
                        new Pose2d(-50, -15.5, Math.toRadians(270)),
                        Math.toRadians(270)
                )

                // Pick up the second wobble goal.
                .addDisplacementMarker(this::obtainWobbleGoal)

                // 5) Spline to starter stack.
                .splineTo(new Vector2d(-28.5, -36.5), Math.toRadians(0))

                // Run intake mechanism to collect 1 ring.
                .addDisplacementMarker(this::activateIntake)
                .forward(intakeDistance)
                .addDisplacementMarker(this::deactivateIntake)

                // 6) Spline to shooting point.
                .splineTo(new Vector2d(-12, -36.5), Math.toRadians(0))

                // Ready shooter motors.
                .addTemporalMarker(shooterTimeMiddleZone2, this::activateShooters)

                // Shoot 3 rings into high goal.
                .addDisplacementMarker(() -> {
                    shootHighGoals();
                    deactivateShooters();
                })

                // 7) Spline to 2nd drop-off point at middle zone.
                .splineTo(new Vector2d(15, -43.5), Math.toRadians(0))

                // Drop the second wobble goal and park.
                .addDisplacementMarker(this::dropWobbleGoal)


                // TODO Remove after testing, will mess up auto path.
                .splineToSplineHeading(
                        new Pose2d(-62, -50, Math.toRadians(0)),
                        Math.toRadians(180)
                )

                // Return trajectory.
                .build();
    }

    /**
     * Goes to closest square if there are 0 rings.
     */
    private void farZone() {
        // Build trajectory.
        trajectory = drive.trajectoryBuilder(new Pose2d(-62, -50)) // 1) Start.
                // 2) Spline to powershot position.
                .splineTo(new Vector2d(-1, -50), Math.toRadians(30))

                // Ready shooter motors.
                .addTemporalMarker(shooterPrepTime1, this::activateShooters)

                // Shoot 3 powershots.
                .addDisplacementMarker(() -> {
                    shootPowershots();
                    deactivateShooters();
                })

                // 3) Spline to 1st drop-off point at far zone.
                .splineTo(new Vector2d(40, -60), Math.toRadians(0))

                // Drop the first wobble goal.
                .addDisplacementMarker(this::dropWobbleGoal)

                // 4) Spline to second wobble goal.
                .splineToLinearHeading(
                        new Pose2d(-50, -15.5, Math.toRadians(270)),
                        Math.toRadians(270)
                )

                // Pick up the second wobble goal.
                .addDisplacementMarker(this::obtainWobbleGoal)

                // 5) Spline to starter stack.
                .splineTo(new Vector2d(-28.5, -36.5), Math.toRadians(0))

                // Run intake mechanism to collect 3 rings.
                .addDisplacementMarker(this::activateIntake)
                .forward(intakeDistance)
                .addDisplacementMarker(this::deactivateIntake)

                // 6) Spline to shooting point.
                .splineTo(new Vector2d(-12, -36.5), Math.toRadians(0))

                // Ready shooter motors.
                .addTemporalMarker(shooterPrepTimeFarZone2, this::activateShooters)

                // Shoot 3 rings into high goal.
                .addDisplacementMarker(() -> {
                    shootHighGoals();
                    deactivateShooters();
                })

                // 7) Spline to 2nd drop-off point at far zone.
                .splineTo(new Vector2d(59, -40), Math.toRadians(270))

                // Drop the second wobble goal.
                .addDisplacementMarker(this::dropWobbleGoal)

                // 8) Spline to parking point.
                .splineTo(new Vector2d(11, -30), Math.toRadians(90))

                // TODO Remove after testing, will mess up auto path.
                .splineToSplineHeading(
                        new Pose2d(-62, -50, Math.toRadians(0)),
                        Math.toRadians(180)
                )

                // Return trajectory.
                .build();
    }

    /**
     * Run intake mechanism.
     */
    private void activateIntake() {
        // TODO Sort out entire intake and high goal code,
        // TODO Set hopper down when intaking

        // Set hopper down.
        hopper.setPosition(hopperInputAngle);

        // Run intake mechanism.
        intake.setPower(intakePowerSetting);
    }

    /**
     * Turn off intake mechanism.
     */
    private void deactivateIntake() {
        // Stop intake mechanism.
        intake.setPower(0);

        // Set hopper up.
        hopper.setPosition(hopperOutputAngle);
    }

    /**
     * Shoot all 3 powershots.
     */
    private void shootPowershots() {
        for (int i = 0; i < 3; i++) {
            // Shoot a ring.
            shoot();

            // Turn 5° counterclockwise.
            drive.turn(Math.toRadians(powershotAngle));
        }
    }

    /**
     * Shoot all 3 high goals..
     */
    private void shootHighGoals() {
        for (int i = 0; i < 3; i++) {
            // Shoot a ring.
            shoot();
        }
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
     * Drop wobble goal.
     */
    private void dropWobbleGoal() {
        // TODO Open elbow before parking.
        elbow.setPosition(elbowDownPosition);
        jaw.setPosition(jawOpenPosition);
    }

    /**
     * Pick up wobble goal.
     */
    private void obtainWobbleGoal() {
        // TODO Close elbow before shooting.
        elbow.setPosition(elbowDownPosition);
        jaw.setPosition(jawClosePosition);
        elbow.setPosition(elbowUpPosition);
    }

    /**
     * Ready shooter motors.
     */
    private void activateShooters() {
        shooter1.setPower(shooterPowerSettingLow);
        shooter2.setPower(shooterPowerSettingLow);
    }

    /**
     * Turn off shooter motors.
     */
    private void deactivateShooters() {
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(double i) {
        // Display label info.
        telemetry.addData("label " + i, recognition.getLabel());

        // Display upper corner info.
        telemetry.addData("Left, Top " + i,
                recognition.getLeft() + recognition.getTop());

        // Display lower corner info.
        telemetry.addData("Right, Bottom " + i,
                recognition.getRight() + recognition.getBottom());

        // Determine Target Zone.
        if (recognition.getLabel().equals("Single")) {
            telemetry.addData("TargetZone", "B");
            wobbleGoalZone = 2;
        } else if (recognition.getLabel().equals("Quad")) {
            telemetry.addData("TargetZone", "C");
            wobbleGoalZone = 3;
        } else {
            telemetry.addData("TargetZone", "UNKNOWN");
        }

        // Update telemetry.
        telemetry.update();
    }
}
