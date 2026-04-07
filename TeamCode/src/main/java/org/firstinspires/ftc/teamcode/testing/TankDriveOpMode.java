package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TankDriveOpMode extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Reverse one side so forward stick = forward robot
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double leftPower = -gamepad1.left_stick_y;   // up is negative
        double rightPower = -gamepad1.right_stick_y;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }
}
