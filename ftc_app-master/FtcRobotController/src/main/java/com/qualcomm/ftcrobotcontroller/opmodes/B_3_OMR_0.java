/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import RobotSquad.RSRobot;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class B_3_OMR_0 extends LinearOpMode {
  RSRobot robot;

  DcMotor motorFrontRight;
  DcMotor motorFrontLeft;
  DcMotor motorBackRight;
  DcMotor motorBackLeft;
  DcMotor motorSlide;
  DcMotor motorHarvester;
  DcMotor motorSpinner;
  Servo servoChurro;
  Servo servoHopper;

  double servoChurroUp = 0.15;
  double servoHopperPos = .5;

  DcMotorController motorControllerFrontDrive;
  DcMotorController motorControllerRearDrive;
  GyroSensor gyro;

  //GyroThread gyrothread;

  @Override
  public void runOpMode() throws InterruptedException {

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

    motorControllerFrontDrive = hardwareMap.dcMotorController.get("frontdrive");
    motorControllerRearDrive = hardwareMap.dcMotorController.get("reardrive");


    gyro = hardwareMap.gyroSensor.get("gyro");

    robot = new RSRobot(gyro);

    //This lets the robot know what way to spin based on alliance
    robot.setMyAlliance(RSRobot.Alliance.BLUE);

    // pass motor objects to robot
    robot.SetFrontRightMotor(motorFrontRight);
    robot.SetFrontLeftMotor(motorFrontLeft);
    robot.SetBackRightMotor(motorBackRight);
    robot.SetBackLeftMotor(motorBackLeft);
    robot.setMotorControllerFrontDrive(motorControllerFrontDrive);
    robot.setMotorControllerRearDrive(motorControllerRearDrive);
    robot.SetHarvesterMotor(motorHarvester);
    robot.SetSpinnerMotor(motorSpinner);

    robot.setOpMode(this);

    // calibrate gyro etc.
    robot.Initialize();

    robot.initializeHarvester();

    waitForStart();
    robot.startHarvester();
// run the autonomous mission
    robot.auto_3_OMC(RSRobot.Alliance.BLUE, 0);

   // robot.DriveForward(1.0, 90);
   // robot.SpinRight(1, 135);
  }
}
