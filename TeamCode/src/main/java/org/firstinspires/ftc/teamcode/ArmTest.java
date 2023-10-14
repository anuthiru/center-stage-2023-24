package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ArmTest extends LinearOpMode {
    //Hi doruk :)
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        Servo grip = hardwareMap.servo.get("grip");
        Servo arm = hardwareMap.servo.get("arm");
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE );

        double target = 10;


        waitForStart();
        double initialGripPos = grip.getPosition();

        if (isStopRequested()) return;

        arm.resetDeviceConfigurationForOpMode();
        arm.getPosition();
        arm.setPosition(0.5);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.a){
                grip.setPosition(0.615);
            }
            if (gamepad1.b){
                grip.setPosition(0.57);
            }





            double error = target - distance.getDistance(DistanceUnit.CM);

            //frontLeft.setPower(frontLeftPower);
            //backLeft.setPower(backLeftPower);
            //frontRight.setPower(frontRightPower);
            //backRight.setPower(backRightPower);

        }

    }
}