package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Autonomous(name = "parking")
//@Disabled
public class parking extends LinearOpMode {
    firstdrive robot;

    int capPosition;
    private ElapsedTime runtime = new ElapsedTime();




    @Override
    public void runOpMode() throws InterruptedException {
        robot = new firstdrive();
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.FLdrive.setDirection(DcMotor.Direction.FORWARD);
            robot.FRdrive.setDirection(DcMotor.Direction.REVERSE);
            robot.moveForwardTo(.7, 10);

        }

    }}