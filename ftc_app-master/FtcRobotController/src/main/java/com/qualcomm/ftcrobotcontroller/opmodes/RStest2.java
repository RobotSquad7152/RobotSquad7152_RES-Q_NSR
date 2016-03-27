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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import RobotSquad.RSRobot;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class RStest2 extends LinearOpMode
{
//Spin testing



//Sensor Testing:

    //GyroSensor gyro;
    OpticalDistanceSensor ods;
    ColorSensor color;

    //GyroThread gyrothread;

    @Override
    public void runOpMode() throws InterruptedException
    {





//        gyro = hardwareMap.gyroSensor.get("gyro");
//        gyro.calibrate();
//        while (gyro.isCalibrating())
//        {
//            try
//            {
//                sleep(5);
//            } catch (InterruptedException e)
//            {
//                e.printStackTrace();
//            }
//        }
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        color = hardwareMap.colorSensor.get("color");

        waitForStart();

        ods.enableLed(true);
        color.enableLed(true);

  //      int i = 0;

        while (opModeIsActive())

        {
            telemetry.addData("ods, ", ods.getLightDetected());
            telemetry.addData("ods raw, ", ods.getLightDetectedRaw());
            telemetry.addData("Red, ", color.red());
            telemetry.addData("Blue, ", color.blue());
            telemetry.addData("Green, ", color.green());

 //          telemetry.addData("i, ", i);
//            if (i++ % 1000 == 0)
//            {
//                gyro.resetZAxisIntegrator();
//            }
            try
            {
                sleep(5);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        ods.enableLed(false);

    }

}

