package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.SimpleDateFormat;
import java.util.Date;

import RobotSquad.RSRobot;

/**
 * Created by Tony on 2/16/2016.
 */
public abstract class RSLinearOpMode extends LinearOpMode
{
    public RSRobot robot;

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorSlide;
    public DcMotor motorHarvester;
    public DcMotor motorSpinner;
    public Servo servoChurro;
    public Servo servoHopper;
    public Servo servoClimber;

    public DcMotorController motorControllerFrontDrive;
    public DcMotorController motorControllerRearDrive;
    public GyroSensor gyro;
    ColorSensor sensorRGB;
    public double servoChurroUp = 0.15;
    public double servoHopperPos = .5;
    public double servoCimberClose = 0;

    @Override
    public abstract void runOpMode() throws InterruptedException;

    public RSRobot InitHardware()
    {
        //initialize motors
        motorFrontLeft = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight = hardwareMap.dcMotor.get("motor_3");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorSlide = hardwareMap.dcMotor.get("motor_5");
        motorSlide.setDirection(DcMotor.Direction.FORWARD);

        motorHarvester = hardwareMap.dcMotor.get("motor_8");
        motorHarvester.setDirection(DcMotor.Direction.FORWARD);

        motorSpinner = hardwareMap.dcMotor.get("motor_7");
        motorSpinner.setDirection(DcMotor.Direction.FORWARD);

        servoChurro = hardwareMap.servo.get("servo_churro");
        servoChurro.setPosition(servoChurroUp);

        servoHopper = hardwareMap.servo.get("servo_hopper");
        servoHopper.setPosition(servoHopperPos);

        servoClimber = hardwareMap.servo.get("servo_climber");
        servoClimber.setPosition(servoCimberClose);

        motorControllerFrontDrive = hardwareMap.dcMotorController.get("frontdrive");
        motorControllerRearDrive = hardwareMap.dcMotorController.get("reardrive");


        gyro = hardwareMap.gyroSensor.get("gyro");



        robot = new RSRobot(gyro);
        sensorRGB = hardwareMap.colorSensor.get("mr");

        // pass motor objects to robot
        robot.SetFrontRightMotor(motorFrontRight);
        robot.SetFrontLeftMotor(motorFrontLeft);
        robot.SetBackRightMotor(motorBackRight);
        robot.SetBackLeftMotor(motorBackLeft);
        robot.setMotorControllerFrontDrive(motorControllerFrontDrive);
        robot.setMotorControllerRearDrive(motorControllerRearDrive);
        robot.SetHarvesterMotor(motorHarvester);
        robot.SetSpinnerMotor(motorSpinner);
        robot.SetColorSensor(sensorRGB);

        robot.setOpMode(this);

        // calibrate gyro etc.
        robot.Initialize();

        robot.initializeHarvester();

        return robot;
    }
}


