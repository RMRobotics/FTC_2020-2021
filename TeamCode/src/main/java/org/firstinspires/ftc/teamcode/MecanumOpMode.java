package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mecanum Op Mode")
public class MecanumOpMode extends LinearOpMode {
    DcMotor intake;
    DcMotor shooter1;
    DcMotor shooter2;
    DcMotor transfer;
    Servo hopperServo;

    Servo jaw;
    Servo elbow;

    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor blMotor;
    DcMotor brMotor;

    Servo indexer;

    /*
    TODO:
    Lower the elbow down position
    make hopper go down when intake same thing with the sequencer
    put a slower shooter button
    give both dpads slow bot movement

     */
    @Override
    public void runOpMode() throws InterruptedException
    {

        //mapping motors
        intake      = hardwareMap.dcMotor.get("intake");
        shooter1    = hardwareMap.dcMotor.get("shooter1");
        shooter2    = hardwareMap.dcMotor.get("shooter2");
        transfer    = hardwareMap.dcMotor.get("transfer");
        flMotor     = hardwareMap.dcMotor.get("fl");
        frMotor     = hardwareMap.dcMotor.get("fr");
        blMotor     = hardwareMap.dcMotor.get("bl");
        brMotor     = hardwareMap.dcMotor.get("br");

        //mapping servos
        jaw         = hardwareMap.servo.get("jaw");
        elbow       = hardwareMap.servo.get("elbow");
        hopperServo = hardwareMap.servo.get("hopper");
        indexer     = hardwareMap.servo.get("indexer");

        //initializing default servo value
        double elbowUpAngle         = 0.1;
        double elbowDownAngle       = 0.4;
        double jawOpenAngle         = 0.1;
        double jawClosedAngle       = 0.36;
        double hopperInputAngle     = 0.1;
        double hopperOutputAngle    = 0.29;
        double indexerUpAngle       = 0.3;
        double indexerDownAngle     = 0;
        double elbowPosition        = elbowUpAngle;
        double jawPosition          = jawClosedAngle;
        double hopperPosition       = hopperInputAngle;
        double indexerPosition      = indexerDownAngle;

        //setting shooters to use encoders
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            double intakePower      = gamepad1.right_trigger;
            double shooterPower     = -gamepad2.right_trigger;
            double transferPower    = gamepad1.left_trigger;

            boolean gamepad2IsDominant = gamepad2.right_trigger > gamepad1.right_trigger;
            boolean rightTriggerOnePressed = gamepad1.right_trigger < .1;
            boolean leftTriggerOnePressed = gamepad1.left_trigger > .1;

            // TODO When press right trigger, shoot 3.
            // TODO Make indexer travel less.
            // TODO Fix the indexer position

            if (gamepad1.left_trigger > .2){
                hopperPosition = hopperInputAngle;
                indexerPosition = indexerDownAngle;
                transferPower = gamepad1.left_trigger;
                intakePower = gamepad1.left_trigger;
            }
            if (gamepad1.right_trigger > .2){
                transferPower = -gamepad1.left_trigger;
                intakePower = -gamepad1.left_trigger;
            }
            if (gamepad2.right_bumper)
            {
                indexerPosition = indexerUpAngle;
            }
            else if (gamepad2.left_bumper)
            {
                indexerPosition = indexerDownAngle;
            }

            flMotor.setDirection(DcMotor.Direction.REVERSE);
            blMotor.setDirection(DcMotor.Direction.REVERSE);

            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;


            if (gamepad1.x) {
                hopperPosition = hopperInputAngle;
                indexerPosition = indexerDownAngle;
            }
            else if (gamepad1.y) hopperPosition = hopperOutputAngle;

            if (gamepad2.a)  elbowPosition = elbowDownAngle;
            else if (gamepad2.b) elbowPosition = elbowUpAngle;

            if (gamepad2.x) jawPosition = jawClosedAngle;
            else if (gamepad2.y) jawPosition = jawOpenAngle;

            double[] speeds = {
                    (drive + strafe +twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            double max = Math.abs(speeds[0]);
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }

            if (max > 1)
            {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }
            if (gamepad2.dpad_up || gamepad1.dpad_up) {
                flMotor.setPower(-.25);
                frMotor.setPower(-.25);
                brMotor.setPower(-.25);
                blMotor.setPower(-.25);
            }
            else if (gamepad2.dpad_right || gamepad1.dpad_right){
                flMotor.setPower(-.25);
                frMotor.setPower(.25);
                brMotor.setPower(.25);
                blMotor.setPower(-.25);
            }
            else if (gamepad2.dpad_left || gamepad1.dpad_left){
                flMotor.setPower(.25);
                frMotor.setPower(-.25);
                brMotor.setPower(-.25);
                blMotor.setPower(.25);
            }
            else if (gamepad2.dpad_down || gamepad1.dpad_down){
                flMotor.setPower(.25);
                frMotor.setPower(.25);
                brMotor.setPower(.25);
                blMotor.setPower(.25);
            }
            else {
                flMotor.setPower(speeds[0]);
                frMotor.setPower(speeds[1]);
                blMotor.setPower(speeds[2]);
                brMotor.setPower(speeds[3]);
            }



            indexer.setPosition(indexerPosition);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            intake.setPower(intakePower);

            transfer.setPower(transferPower);

            hopperServo.setPosition(hopperPosition);
            elbow.setPosition(elbowPosition);
            jaw.setPosition(jawPosition);

            telemetry.addData("pressing x: ", "value: " + gamepad2.x);
            telemetry.addData( "pressing y", "value: " + gamepad2.y);
            telemetry.addData("Motors", "Y Power " + gamepad1.left_stick_y);
            telemetry.addData( "Motors", "Y Power " + gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
