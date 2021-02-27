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
import java.util.Vector;

@Autonomous(name="AutoOpMode (Red)")
public class AutoOpMode1 extends LinearOpMode {
    // Data field.
    private VuforiaCurrentGame vuforiaUltimateGoal;
    private TfodCurrentGame tfodUltimateGoal;
    private SampleMecanumDrive drive;

    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor transfer;
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
    double intakeOnSetting;
    double intakeOffSetting;

    double transferOnSetting;
    double transferOffSetting;
    double indexerUpPosition;
    double indexerDownPosition;
    double elbowUpPosition;
    double elbowDownPosition;
    double hopperInputAngle;
    double hopperOutputAngle;
    double jawOpenPosition;
    double jawClosePosition;

    double intakeDistance;
    double openIntakePosition;
    double cameraPosition;
    double elbowBackPosition;
    double powershotAngle;
    double powershotRotation;
    double highGoalAngle;

    boolean doingIntake;

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
        transfer            = hardwareMap.get(DcMotor.class, "transfer");
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
        shooterPowerSettingHigh     = 0.6;
        shooterPowerSettingLow      = 0.53;
        transferOnSetting           = 1;
        transferOffSetting          = 0;
        intakeOnSetting             = 1;
        intakeOffSetting            = 0;
        indexerUpPosition           = 0.3;      // TODO fix wth robot.
        indexerDownPosition         = 0;        // TODO fix wth robot.
        elbowUpPosition             = 0.3;
        elbowDownPosition           = 0.71;
        hopperInputAngle            = 0.1;      // TODO Hopper up position = 0.29
        hopperOutputAngle           = 0.29;     // TODO Hopper down position = 0.1
        jawOpenPosition             = 0.1;      // TODO set position for jaw, also needs tele-op
        jawClosePosition            = 0.38;     // TODO set position for jaw, also needs tele-op

        intakeDistance              = 5;        // TODO fix with robot, currently set to 0.
        openIntakePosition          = 0.6;
        cameraPosition              = 0.065;
        elbowBackPosition           = 0;
        powershotAngle              = 20;
        powershotRotation           = 4;
        highGoalAngle               = 7;

        doingIntake                 = false;

        // Configure all devices before operation.
        jaw.setPosition(jawClosePosition);
        elbow.setPosition(elbowBackPosition);
        hopper.setPosition(hopperOutputAngle);
        cameraServo.setPosition(cameraPosition);
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        drive.setPoseEstimate(new Pose2d(-62, -50));
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
                telemetry.update();
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

            // Update telemetry.
            telemetry.update();

            // Stop if requested.
            if (isStopRequested()) return;

            // Determine trajectory.
            // TODO CHANGED, put == 1 first (before it was last) and added telemetry.update().
            if (wobbleGoalZone == 1){
                telemetry.addData("TargetZone", "C");
                telemetry.update();
                closeZone();
            } else if (wobbleGoalZone == 2) {
                telemetry.addData("TargetZone", "B");
                telemetry.update();
                middleZone();
            } else if (wobbleGoalZone == 3) {
                telemetry.addData("TargetZone", "A");
                telemetry.update();
                farZone();
            }
        }

        // Close tensorflow engine.
        tfodUltimateGoal.close();
    }

    /**
     * Goes to farthest square if there are 4 rings.
     */
    private void closeZone() {
        /*  Program
                Start at A (-62, -52)
                Go to B (-1, -50)
                    Shoot and reset angle
                    Drop wobble and reset angle
                Go to C (-35, -20)
                    Pick wobble
                Go to D (-10, -58)
                    Drop Wobble
                Go to E (11, -30) to park
         */

        // Trajectory 1 to shoot and drop wobble
        // Start at point A (-62, -50)
        // Go to point B (-1, -50)
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(-62, -50)) // 1) Start.
                // 2) Spline to high goal shooting position.
                .splineTo(new Vector2d(-20, -30), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory1);

        // Shoot 3 rings into high goal.
        activateShooters(true);
        shootHighGoals(3);
        deactivateShooters();

        /*/ Shoot 3 powershots (pegs) with moves 5° each, back to same heading as start.
        activateShooters(false);
        shootPowershots();
        deactivateShooters();*/

        // Trajectory 2.
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                // 3) Spline to first drop-off point.
                .splineTo(new Vector2d(-1, -50), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory2);

        // Move, drop the wobble, reset back to same heading.
        drive.turn(Math.toRadians(-60));
        dropWobbleGoal();
        drive.turn(Math.toRadians(60)); // Reset to heading 0°.

        // Trajectory 2 to pick wobble
        // Go to point C (-35, -20) to pick wobble
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                // 4) Spline to second wobble goal.
                .splineTo(new Vector2d(-37, -22), Math.toRadians(180))

                .build();
        drive.followTrajectory(trajectory3);

        // Pick up the second wobble goal.
        obtainWobbleGoal();

        // Trajectory 4 to drop wobble and park.
        // Go to point D (-10, -58) to drop wobble.
        // Go to point E (11, -30) to Park.
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                // 5) Spline to 2nd drop-off point at close zone.
                .splineTo(new Vector2d(-10, -58), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory4);

        // Drop wobble goal.
        drive.turn(Math.toRadians(-45));
        dropWobbleGoal();

        Trajectory trajectory5_1 = drive.trajectoryBuilder(
                new Pose2d(-10, -58, Math.toRadians(-45)))

                .splineToConstantHeading(new Vector2d(-15, -58), Math.toRadians(180))

                .build();
        drive.followTrajectory(trajectory5_1);

        // Turn necessary degrees.
        drive.turn(Math.toRadians(45));
        drive.turn(Math.toRadians(90));

        // Trajectory 5 to park.
        // Go to point E (11, -30) to Park.
        Trajectory trajectory5 = drive.trajectoryBuilder(
                new Pose2d(-15, -58, Math.toRadians(90)))
                // 5) Spline to parking point.
                .splineTo(new Vector2d(11, -30), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory5);

        sleep(5000);

        // Trajectory 6.
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                // TODO Remove after, splines back to start.
                // .splineToSplineHeading(new Pose2d(-62, -50), Math.toRadians(180))

                .build();
        drive.followTrajectory(trajectory6);
    }

    /**
     * Goes to middle square if there is 1 ring.
     */
    private void middleZone() {
    /*  Program
            Start at A (-62, -50)
            Go to B (-1, -50)
                Shoot high goals and reset angle
            Go to C (15, -33)
                Drop wobble and reset angle
            Go to D (-35, -20)
                Pick wobble
            Go to E (-28.5, -36.5)
                Intake
            Go to F (-12, -36.5)
                Shoot high goals
            Go to G (15, -43.5) to Park
                Drop Wobble
                Park
     */

        // Trajectory 1.
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(-62, -50)) // 1) Start.
                // 2) Spline to powershot position.
                .splineTo(new Vector2d(-22, -50), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory1);

        /*/ Shoot 3 powershots (pegs) with moves 5° each, back to same heading as start.
        activateShooters(false);
        shootPowershots();
        deactivateShooters();*/

        // Shoot 3 rings into high goal.
        drive.turn(Math.toRadians(highGoalAngle));
        activateShooters(true);
        shootHighGoals(3);
        deactivateShooters();
        drive.turn(Math.toRadians(-highGoalAngle));

        // Trajectory 2.
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                // 3) Spline to 1st drop-off point at middle zone.
                .splineTo(new Vector2d(15, -33), Math.toRadians(90))

                .build();
        drive.followTrajectory(trajectory2);

        // Drop the first wobble goal.
        drive.turn(Math.toRadians(-90));
        dropWobbleGoal();
        drive.turn(Math.toRadians(90));

        // Trajectory 3.
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                // 4) Spline to second wobble goal.
                .splineTo(new Vector2d(-37, -22), Math.toRadians(180))

                .build();
        drive.followTrajectory(trajectory3);

        // Pick up the second wobble goal.
        obtainWobbleGoal();

        // Activate intake mechanism.
        activateIntake();

        // Trajectory 4.
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                // 5) Spline to starter stack.
                .splineTo(new Vector2d(-20, -35.5), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory4);

        // Trajectory 5.
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                // 6) Spline to shooting point.
                .splineToConstantHeading(new Vector2d(-7, -35.5), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory5);

        // Deactivate intake mechanism.
        deactivateIntake();

        // Shoot 3 rings into high goal.
        activateShooters(true);
        shootHighGoals(1);
        deactivateShooters();

        // Trajectory 6.
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                // 7) Spline to 2nd drop-off point at middle zone.
                .splineTo(new Vector2d(14, -43.5), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory6);

        // Drop second wobble goal.
        dropWobbleGoal();

        // Trajectory 7
        // Drop the second wobble goal and park.
        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .splineToConstantHeading(new Vector2d(7, -43.5), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory7);

        sleep(5000);

        // Trajectory 8.
        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                // TODO Remove after, splines back to start.
                // .splineToSplineHeading(new Pose2d(-62, -50), Math.toRadians(180))

                .build();
        drive.followTrajectory(trajectory8);
    }


    /**
     * Goes to closest square if there are 0 rings.
     */
    private void farZone() {
        /*  Program
                Start at A (-62, -52)
                Go to B (-22, -50)
                    Shoot and reset angle
                Go to C (40, -60)
                    Drop wobble and reset angle
                Go to D (-37, -22)
                    Pick wobble
                Go to E (-20, -35.5)
                    Intake
                Go to F (-7, -35.5)
                    Shoot high goals
                Go to G (50, -40)
                    Drop wobble
                Go to H (11, -30) to park.
         */

        // Trajectory 1.
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d(-62, -50)) // 1) Start.
                // 2) Spline to powershot position.
                .splineTo(new Vector2d(-22, -50), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory1);

        /*/ Shoot 3 powershots.
        activateShooters(false);
        shootPowershots();
        deactivateShooters();*/

        // Shoot 3 rings into high goal.
        drive.turn(Math.toRadians(highGoalAngle));
        activateShooters(true);
        shootHighGoals(3);
        deactivateShooters();
        drive.turn(Math.toRadians(-highGoalAngle));

        // Trajectory 2.
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                // 3) Spline to 1st drop-off point at far zone.
                .splineTo(new Vector2d(40, -62), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory2);

        // Drop the first wobble goal.
        drive.turn(Math.toRadians(-30));
        dropWobbleGoal();
        drive.turn(Math.toRadians(30));

        // Trajectory 3.1.
        Trajectory trajectory2_1 = drive.trajectoryBuilder(trajectory2.end())
                // Move back.
                .splineToConstantHeading(new Vector2d(35, -62), Math.toRadians(0))

                .build();
        drive.followTrajectory(trajectory2_1);

        // Turn 90°.
        drive.turn(Math.toRadians(90));

        // Trajectory 3.
        Trajectory trajectory3 = drive.trajectoryBuilder(
                new Pose2d(35, -62, Math.toRadians(90)))

                // 4) Spline to second wobble goal.
                .splineTo(new Vector2d(-37, -21), Math.toRadians(180))

                .build();
        drive.followTrajectory(trajectory3);

        // Pick up the second wobble goal.
        obtainWobbleGoal();

        // If doing powershots.
        if (doingIntake) {
            // Trajectory 4.
            Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                    // 5) Spline to starter stack.
                    .splineTo(new Vector2d(-20, -35.5), Math.toRadians(0))

                    .build();
            drive.followTrajectory(trajectory4);

            // Trajectory 4.1.
            Trajectory trajectory4_1 = drive.trajectoryBuilder(trajectory4.end())
                    // Go backward.
                    .splineToConstantHeading(new Vector2d(-25, -35.5), Math.toRadians(0))

                    .build();
            drive.followTrajectory(trajectory4_1);

            // Activate intake.
            activateIntake();

            // Trajectory 5
            Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4_1.end())
                    // 6) Spline to shooting point.
                    .splineToConstantHeading(new Vector2d(-7, -35.5), Math.toRadians(0))

                    .build();
            drive.followTrajectory(trajectory5);

            // Deactivate intake mechanism.
            sleep(1000);
            deactivateIntake();

            /*/ Shoot 3 rings into high goal.
            activateShooters(true);
            shootHighGoals(3);
            deactivateShooters();*/

            // Trajectory 6.
            Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                    // 7) Spline to 2nd drop-off point at far zone.
                    .splineTo(new Vector2d(50, -45), Math.toRadians(270))

                    .build();
            drive.followTrajectory(trajectory6);
        }

        // If skipping powershots.
        else {
            // Turn 180°.
            drive.turn(Math.toRadians(180));

            // Trajectory 6.
            Trajectory trajectory6 = drive.trajectoryBuilder(new Pose2d(-37, -21))
                    // 7) Spline to 2nd drop-off point at far zone.
                    .splineTo(new Vector2d(50, -45), Math.toRadians(270))

                    .build();
            drive.followTrajectory(trajectory6);
        }

        // Drop the second wobble goal.
        dropWobbleGoal();

        // Trajectory 5.1.
        Trajectory trajectory5_1 = drive.trajectoryBuilder(
                new Pose2d(50, -45, Math.toRadians(270)))

                .splineToConstantHeading(new Vector2d(50, -40), Math.toRadians(90))

                .build();
        drive.followTrajectory(trajectory5_1);

        // Trajectory 7.
        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory5_1.end())
                // 8) Spline to parking point.
                .splineTo(new Vector2d(11, -30), Math.toRadians(90))

                .build();
        drive.followTrajectory(trajectory7);

        sleep(5000);

        // Trajectory 8.
        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
                // TODO Remove after, splines back to start.
                // .splineToSplineHeading(new Pose2d(-62, -50), Math.toRadians(180))

                .build();
        drive.followTrajectory(trajectory8);
    }

    /**
     * Shoot all 3 powershots.
     */
    private void shootPowershots() {
        // Initialize variables.
        int restorationAngle = 0;
        int decrementAngle = 0;

        for (int i = 0; i < 2; i++) {
            // Shoot a ring.
            shoot();

            // Turn 5° counterclockwise.
            drive.turn(Math.toRadians(powershotRotation));

            // Increment restoration angle.
            restorationAngle++;
        }

        // Shoot the last ring.
        shoot();

        // Calculate restoration angle.
        restorationAngle *= -powershotRotation;

        // Restore angle.
        drive.turn(Math.toRadians(restorationAngle - powershotAngle));
    }

    /**
     * Shoot all 3 high goals..
     */
    private void shootHighGoals(int number) {
        for (int i = 0; i < number; i++) {
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
        sleep(500);
        indexer.setPosition(indexerDownPosition);
        sleep(500);
    }

    /**
     * Drop wobble goal.
     */
    private void dropWobbleGoal() {
        // TODO Open elbow before parking.
        elbow.setPosition(elbowDownPosition);
        sleep(1000);
        jaw.setPosition(jawOpenPosition);
        sleep(250);
    }

    /**
     * Pick up wobble goal.
     */
    private void obtainWobbleGoal() {
        // TODO Close elbow before shooting.
        elbow.setPosition(elbowDownPosition);
        sleep(250);
        jaw.setPosition(jawClosePosition);
        sleep(250);
        elbow.setPosition(elbowUpPosition);
        sleep(250);
    }

    /**
     * Run intake mechanism.
     */
    private void activateIntake() {
        // TODO Sort out entire intake and high goal code,
        // TODO Set hopper down when intaking

        // Set hopper down.
        hopper.setPosition(hopperInputAngle);
        transfer.setPower(transferOnSetting);
        sleep(250);

        // Run intake mechanism.
        cameraServo.setPosition(openIntakePosition);
        intake.setPower(intakeOnSetting);
        sleep(250);
    }

    /**
     * Turn off intake mechanism.
     */
    private void deactivateIntake() {
        // Stop intake mechanism.
        intake.setPower(intakeOffSetting);
        transfer.setPower(transferOffSetting);
        sleep(250);

        // Set hopper up.
        hopper.setPosition(hopperOutputAngle);
        sleep(250);
    }

    /**
     * Ready shooter motors.
     */
    private void activateShooters(boolean highPower) {
        if (highPower) {
            shooter1.setPower(shooterPowerSettingHigh);
            shooter2.setPower(shooterPowerSettingHigh);
        } else {
            shooter1.setPower(shooterPowerSettingLow);
            shooter2.setPower(shooterPowerSettingLow);
        }
        sleep(250);
    }

    /**
     * Turn off shooter motors.
     */
    private void deactivateShooters() {
        indexer.setPosition(indexerDownPosition);
        sleep(250);
        shooter1.setPower(0);
        shooter2.setPower(0);
        sleep(250);
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
    }
}
