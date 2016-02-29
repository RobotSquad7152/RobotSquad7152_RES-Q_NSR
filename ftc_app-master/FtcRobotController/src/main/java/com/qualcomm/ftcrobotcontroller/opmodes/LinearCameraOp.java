package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.ByteArrayOutputStream;

import RobotSquad.CameraPreview;
import RobotSquad.RSCamera;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class LinearCameraOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        int middle = 0;
        RSCamera rsCamera = new RSCamera((FtcRobotControllerActivity)hardwareMap.appContext, telemetry);
        rsCamera.init();
        while (true)
        {
            middle = rsCamera.calculateMiddle();

        }
    }
}