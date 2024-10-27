package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TetsyWetsyUwU extends LinearOpMode {
    int armtarget=0;
    int buckettarget=0;
    int hangtarget=0;
    int extendotarget=0;
    double pitchtarget=135;
    double dynamicKp=0.015;
    boolean isPressingA=false;
    boolean isPressingB=false;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx bucket = hardwareMap.get(DcMotorEx.class, "bucket");
        DcMotorEx hang = hardwareMap.get(DcMotorEx.class, "bucket");
        while (opModeIsActive()){
            if (isStopRequested()) return;
            if (dynamicKp==0.015){
                dynamicKp=0.016;
            }
            else if (dynamicKp==0.016){
                dynamicKp=0.014;
            }
            else if (dynamicKp==0.014){
                dynamicKp=0.015;
            }
        }
    }
    public void armTets(){
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad1.left_trigger>0){
            armtarget-=15;
        }
        else if (gamepad1.right_trigger>0){
            armtarget+=15;
        }
        arm.setPower(dynamicKp*(armtarget-arm.getCurrentPosition()));
    }
    public void bucketTets(){
        DcMotorEx bucket = hardwareMap.get(DcMotorEx.class, "bucket");
        bucket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad1.left_bumper){
            buckettarget-=15;
        }
        else if (gamepad1.right_bumper){
            buckettarget+=15;
        }
        bucket.setPower(dynamicKp*(armtarget-bucket.getCurrentPosition()));
    }
    public void hangTets(){
        DcMotorEx hang = hardwareMap.get(DcMotorEx.class, "hang");
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.left_bumper){
            hangtarget-=15;
        }
        else if (gamepad2.right_bumper){
            hangtarget+=15;
        }
        hang.setPower(dynamicKp*(hangtarget-hang.getCurrentPosition()));
    }
    public void extendoTets(){
        DcMotorEx extendo=hardwareMap.get(DcMotorEx.class, "extendo");
        if (gamepad2.left_trigger>0){
            extendotarget+=15;
        }
        else if (gamepad2.right_trigger>0 && extendotarget>15){
            extendotarget-=15;
        }
        extendo.setPower(dynamicKp*(extendotarget-extendo.getCurrentPosition()));
    }
    public void clawFingerTets(){
        Servo clawfingers = hardwareMap.get(Servo.class, "clawfingers");
        if (gamepad1.a) {
            if (isPressingA) {
                clawfingers.setPosition(1 - clawfingers.getPosition());
                isPressingA=true;
            }
        }
        else{
            isPressingA=false;
        }
    }
    public void clawRollTets(){
        Servo clawroll = hardwareMap.get(Servo.class, "clawroll");
        if (gamepad1.b) {
            if (isPressingB) {
                if(clawroll.getPosition()==0){clawroll.setPosition(0.5);}else{clawroll.setPosition(0);}
                isPressingB=true;
            }
        }
        else{
            isPressingB=false;
        }
    }
    public void clawPitchTets(){
        Servo pitch1 = hardwareMap.get(Servo.class, "clawpitch");
        Servo pitch2 = hardwareMap.get(Servo.class, "clawpitch");
        if (gamepad1.left_stick_y>0&&pitchtarget<210) {
            pitchtarget+=15;
        }
        else if (gamepad1.left_stick_y<0&&pitchtarget>60){
            pitchtarget-=15;
        }
        pitch1.setPosition(pitchtarget/270);
        pitch2.setPosition(pitchtarget/270);
    }
}
