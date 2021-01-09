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
    CRServo jaw;

    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor blMotor;
    DcMotor brMotor;
    CRServo hopperio;

    /*
    elbow is 2 and jaw is 3 on hub 3
     */

    @Override
    public void runOpMode() throws InterruptedException
    {
        elbow = hardwareMap.servo.get("elbow");
        jaw = hardwareMap.crservo.get("jaw");

        intake = hardwareMap.dcMotor.get("intake");
        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");
        transfer = hardwareMap.dcMotor.get("transfer");

        flMotor = hardwareMap.dcMotor.get("fl");
        frMotor = hardwareMap.dcMotor.get("fr");
        blMotor = hardwareMap.dcMotor.get("bl");
        brMotor = hardwareMap.dcMotor.get("br");

        hopperio = hardwareMap.crservo.get("crservo");
        hopperServo = hardwareMap.servo.get("hopperangle");

        double hopperInputAngle = .1;
        double hopperOutputAngle = .29;
        double hopperAngle = hopperInputAngle;
        double elbowAngle = 0;
        double jawPower;
        waitForStart();
        while(opModeIsActive())
        {
            jawPower = 0;
            double intakePower      = -gamepad1.right_trigger;
            double shooterPower     = -gamepad1.left_trigger;
            double hopperPower      = 0;//gamepad1.right_bumper ? 1:0;
            double transferPower    = gamepad1.right_trigger;

            if(gamepad2.right_trigger > gamepad1.right_trigger)
            {
                transferPower = -gamepad2.right_trigger;
                intakePower = gamepad2.right_trigger;
            }
            if (gamepad1.right_trigger < .1)
            {
                hopperPower = gamepad1.right_bumper ? 1 : 0;
            }
            else {
                hopperPower = -gamepad1.right_trigger/2;
            }

            flMotor.setDirection(DcMotor.Direction.REVERSE);
            blMotor.setDirection(DcMotor.Direction.REVERSE);

            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;


            if (gamepad1.x) hopperAngle = hopperInputAngle;
            else if (gamepad1.y) hopperAngle= hopperOutputAngle;

            if ( gamepad2.a)  elbowAngle = 0.3;
            else if ( gamepad2.b) elbowAngle = 0.65;

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

            flMotor.setPower(speeds[0]);
            frMotor.setPower(speeds[1]);
            blMotor.setPower(speeds[2]);
            brMotor.setPower(speeds[3]);

            hopperio.setPower(hopperPower);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            intake.setPower(intakePower);

            transfer.setPower(transferPower);

            hopperServo.setPosition(hopperAngle);
            elbow.setPosition(elbowAngle);
            jaw.setPower(jawPower);
            telemetry.addData("pressing x: ", "value: " + gamepad2.x);
            telemetry.addData( "pressing y", "value: " + gamepad2.y);
            telemetry.addData("Motors", "Y Power " + gamepad1.left_stick_y);
            telemetry.addData( "Motors", "Y Power " + gamepad1.left_stick_x);
            telemetry.update();
        }
    }


}
