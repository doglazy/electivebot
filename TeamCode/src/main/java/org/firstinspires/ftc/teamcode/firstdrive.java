package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;
import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class firstdrive {
    /* Public OpMode members. */
    public DcMotor FRdrive;
    public DcMotor FLdrive;
    public DcMotor BLdrive;
    public DcMotor BRdrive;
    public WebcamName camera;
    public Servo bucket;
    public RevColorSensorV3 color;
    public Rev2mDistanceSensor distance;
    public RevColorSensorV3 colorR;
    public RevColorSensorV3 colorL;
    public BNO055IMU imu;


    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.543;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public int arm_start;
    public int platform_start;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    int nx = 0;
    int ny = 0;
    int sumX = 0;
    int[] RightOG = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int[] LeftOG = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int sumY = 0;


    /* Constructor */
    public firstdrive() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FRdrive = hwMap.dcMotor.get("Front right");
        FLdrive = hwMap.dcMotor.get("Front left");
        BLdrive = hwMap.dcMotor.get("Rear left");
        BRdrive = hwMap.dcMotor.get("Rear right");


        // Set motors' direction
        FRdrive.setDirection(DcMotor.Direction.FORWARD);
        FLdrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        FRdrive.setPower(0);
        FLdrive.setPower(0);
        //carousel.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FRdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //bucket = hwMap.servo.get("bucket");

        // Set sensors
        /*camera = hwMap.get(WebcamName.class, "camera");
        color = hwMap.get(RevColorSensorV3.class, "color");
        distance = hwMap.get(Rev2mDistanceSensor.class, "distance");
        colorR = hwMap.get(RevColorSensorV3.class, "colorR");
        colorL = hwMap.get(RevColorSensorV3.class, "colorL");

         */

        //imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



    }

    //stop drive motors
    public void stopMoving() {
        FRdrive.setPower(0);
        FLdrive.setPower(0);
        BLdrive.setPower(0);
        BRdrive.setPower(0);

    }

    public void moveForwardTo(double power, double inches) {
        int position = inchesToPosition(inches);
        AddDataX(RightOG, -position);
        AddDataY(LeftOG, -position);
        startMovingToPosition(power);
    }

    public void moveBackwardTo(double power, double inches) {
        int position = inchesToPosition(inches);
        AddDataX(RightOG, position);
        AddDataY(LeftOG, position);
        startMovingToPosition(power);
    }

    public void spinLeft(double power, double inches) {
        int position = inchesToPosition(inches);
        AddDataX(RightOG, position );
        AddDataY(LeftOG, -position);
        startMovingToPosition(power);
    }

    public void spinRight(double power, double inches) {
        int position = inchesToPosition(inches);
        AddDataX(RightOG, -position );
        AddDataY(LeftOG, position);
        startMovingToPosition(power);
    }


    protected int getTestDataX() {
        int i;
        sumX = 0;
        for (i = 0; i < RightOG.length; i++) {
            sumX += RightOG[i];
        }
        return sumX;
    }
    protected int getTestDataY() {
        int i;
        sumY = 0;
        for (i = 0; i < LeftOG.length; i++) {
            sumY += LeftOG[i];
        }
        return sumY;
    }


    private void startMovingToPosition(double pow) {
        setMotorPosition();
        setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(pow);
        /*FLDriveMoving = true;
        BLDriveMoving = true;
        FRDriveMoving = true;
        BRDriveMoving = true;
         */
        while (isRobotMoving()) { }
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPower(0);

    }
    private void setMotorPosition() {
        getTestDataX();
        getTestDataY();
        FLdrive.setTargetPosition(sumY);
        FRdrive.setTargetPosition(sumX);
    }

    public void setMotorsMode(DcMotor.RunMode mode) {
        FRdrive.setMode(mode);
        FLdrive.setMode(mode);
    }


    public boolean isRobotMoving() {
        return FLdrive.isBusy() || FRdrive.isBusy();


        /*if (FLDriveMoving)
            FLDriveMoving = !isAtTargetPosition(FLdrive.getCurrentPosition(), FLdrive.getTargetPosition());
        if (FRDriveMoving)
            FRDriveMoving = !isAtTargetPosition(FRdrive.getCurrentPosition(), FRdrive.getTargetPosition());
        if (BLDriveMoving)
            BLDriveMoving = !isAtTargetPosition(BLdrive.getCurrentPosition(), BLdrive.getTargetPosition());
        if (BRDriveMoving)
            BRDriveMoving = !isAtTargetPosition(BRdrive.getCurrentPosition(), BRdrive.getTargetPosition());
        return FLDriveMoving || FRDriveMoving || BLDriveMoving || BRDriveMoving;
         */
    }

    private boolean isAtTargetPosition(int current, int target) {
        return Math.abs(Math.abs(current) - Math.abs(target)) < 1.2;
    }


    public int[] AddDataX(int RightOG[], int DeltaXR){
        RightOG[nx] = DeltaXR;
        nx++;
        return RightOG;
    }
    public int[] AddDataY(int LeftOG[], int DeltaXL){
        LeftOG[ny] = DeltaXL;
        ny++;
        return LeftOG;
    }

    public void correctPos(){
        int correctionleft = (FLdrive.getCurrentPosition() - sumY);
        int correctionright = (FRdrive.getCurrentPosition() - sumX);
        AddDataX(RightOG, correctionright);
        AddDataY(LeftOG, correctionleft);
    }

    private int inchesToPosition(double inches) {
        return (int) (inches * COUNTS_PER_INCH);
    }

    public void setPower(double power) {
        FLdrive.setPower(power);
        FRdrive.setPower(power);
    }


}
