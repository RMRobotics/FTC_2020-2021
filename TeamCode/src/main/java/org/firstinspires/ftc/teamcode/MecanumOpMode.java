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

    Servo elbow;
    Servo jaw;

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
        elbow       = hardwareMap.servo.get("elbow");
        jaw         = hardwareMap.servo.get("jaw");
        hopperServo = hardwareMap.servo.get("hopper");
        indexer     = hardwareMap.servo.get("indexer");

        //initializing default servo value
        double hopperInputAngle = .1;
        double hopperOutputAngle = .29;
        double hopperAngle = hopperInputAngle;
        double elbowAngle = 0;
        double jawPower;
        double indexerPosition = 0;

        //setting shooters to use encoders
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive())
        {
            jawPower = 0;
            //
            double intakePower      = gamepad1.left_trigger;
            double shooterPower     = -gamepad1.right_trigger;
            double transferPower    = gamepad1.left_trigger;

            boolean gamepad2IsDominant = gamepad2.right_trigger > gamepad1.right_trigger;
            boolean rightTriggerOnePressed = gamepad1.right_trigger < .1;
            boolean leftTriggerOnePressed = gamepad1.left_trigger > .1;

            // TODO When press right trigger, shoot 3.
            // TODO Make indexer travel less.
            // TODO Fix the indexer position

            if(gamepad1.left_trigger > .2){
                hopperAngle = hopperInputAngle;
                indexerPosition = 0;
            }

            if(gamepad2IsDominant)
            {
                transferPower = -gamepad2.right_trigger;
                intakePower = -gamepad2.right_trigger;
            }

            if (gamepad1.right_bumper)
            {
                indexerPosition = 126 / 180.0;
            }
            else if (gamepad1.left_bumper)
            {
                indexerPosition = 0;
            }

            flMotor.setDirection(DcMotor.Direction.REVERSE);
            blMotor.setDirection(DcMotor.Direction.REVERSE);

            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;


            if (gamepad1.x) {
                hopperAngle = hopperInputAngle;
                indexerPosition = 0;
            }
            else if (gamepad1.y) hopperAngle= hopperOutputAngle;

            if ( gamepad2.a)  elbowAngle = 0.28;
            else if ( gamepad2.b) elbowAngle = 0.70;

            if (gamepad2.x) jawPower = .5;
            else if ( gamepad2.y) jawPower = -.5;

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

            if (max > 1 )
            {
                for ( int i = 0; i < speeds.length; i++ ) speeds[i] /= max;
            }
            if(gamepad2.dpad_up || gamepad1.dpad_up) {
                flMotor.setPower(-.25);
                frMotor.setPower(-.25);
                brMotor.setPower(-.25);
                blMotor.setPower(-.25);
            }
            else if(gamepad2.dpad_right || gamepad1.dpad_right){
                flMotor.setPower(-.25);
                frMotor.setPower(.25);
                brMotor.setPower(.25);
                blMotor.setPower(-.25);
            }
            else if(gamepad2.dpad_left || gamepad1.dpad_left){
                flMotor.setPower(.25);
                frMotor.setPower(-.25);
                brMotor.setPower(-.25);
                blMotor.setPower(.25);
            }
            else if(gamepad2.dpad_down || gamepad1.dpad_down){
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

            hopperServo.setPosition(hopperAngle);
            elbow.setPosition(elbowAngle);
            jaw.setPosition(jawPower);
            telemetry.addData("pressing x: ", "value: " + gamepad2.x);
            telemetry.addData( "pressing y", "value: " + gamepad2.y);
            telemetry.addData("Motors", "Y Power " + gamepad1.left_stick_y);
            telemetry.addData( "Motors", "Y Power " + gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
