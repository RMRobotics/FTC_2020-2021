package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
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

    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor blMotor;
    DcMotor brMotor;
    CRServo hopperio;


    protected MecanumDrive mecanum;
    @Override
    public void runOpMode() throws InterruptedException
    {
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

        waitForStart();
        while(opModeIsActive())
        {
            double intakePower = -gamepad1.right_trigger;
            double shooterPower = -gamepad1.left_trigger;
            double hopperPower = gamepad1.right_bumper ? 1.0 : 0.0;
            double transferPower = gamepad1.left_bumper ? 1.0 : 0;

            flMotor.setDirection(DcMotor.Direction.REVERSE);
            blMotor.setDirection(DcMotor.Direction.REVERSE);

            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;


            if (gamepad1.square) hopperAngle = hopperInputAngle;
            else if (gamepad1.triangle) hopperAngle= hopperOutputAngle;


            double[] speeds = {
                    (drive + strafe +twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            double max = Math.abs(speeds[0]);
            for( int i = 0; i < speeds.length; i++){
                if ( max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
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

            telemetry.addData("Motors", "Y Power " + gamepad1.left_stick_y);
            telemetry.addData( "Motors", "Y Power " + gamepad1.left_stick_x);
            telemetry.update();
        }
    }


}
