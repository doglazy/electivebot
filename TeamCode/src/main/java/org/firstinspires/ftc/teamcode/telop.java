package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;
import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp")
public class telop extends LinearOpMode {
    firstdrive robot;
    int targetposintake;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new firstdrive();
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
              robot.FRdrive.setDirection(DcMotor.Direction.FORWARD);
              robot.FLdrive.setDirection(DcMotor.Direction.REVERSE);
                robot.FRdrive.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                robot.FLdrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x)*.9);
                robot.BRdrive.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                robot.BLdrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x)*.9);
                while (opModeIsActive() && gamepad1.right_bumper == true){
                    robot.FRdrive.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                    robot.FLdrive.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                    robot.BRdrive.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                    robot.BLdrive.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                }
                telemetry.addData("motordL", robot.FLdrive.getCurrentPosition());
                telemetry.addData("X", gamepad1.left_stick_x);
                telemetry.addData("y", gamepad1.left_stick_y);
                telemetry.update();
                if (gamepad1.square) {
                    robot.FRdrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) / 4);
                    robot.FLdrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) / 4);
                }
            }
        }
    }
}