package RobotSquad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

//import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Tony on 10/4/2015.
 */

public class RSRobot {


    LinearOpMode opMode;
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorHarvester;
    DcMotor motorSpinner;
    DcMotorController motorControllerFrontDrive;
    DcMotorController motorControllerRearDrive;
    private GyroThread gyrothread;
    private GyroSensor gyro;
    private BucketThread bucketThread;

    public enum Alliance{
        BLUE  (1),
        RED  (-1);
        private final double alliance;

        private Alliance(double alliance) {
            this.alliance = alliance;
        }
    }



    Alliance myAlliance;
    public void setMyAlliance(Alliance myAlliance) {
        this.myAlliance = myAlliance;
    }

    //defines drive wheel diameter -- a 2 inch sprocket this year
    final double wheeldiacm = 2.76 * 2.54;
    //defines drive wheel circumference
    final double wheelcirccm = wheeldiacm * 3.141592653589793;
    //defines how many teeth on gear attached to motor (no gearing this year)
    final double motorgearteeth = 3;
    //defines how many teeth on gear attached to wheel (no gearing this year)
    final double wheelgearteeth = 2;
    //encoder counts per rotation of the motor
    final double motorclicksperrotation = 1120;
    //calculates how far the robot will drive for each motor encoder click
    final double onemotorclick = ((motorgearteeth / wheelgearteeth) * wheelcirccm) / motorclicksperrotation;





    public void Initialize() {
        if (gyro != null) {
            gyrothread = new GyroThread(gyro);

            gyrothread.calibrategyro();

            gyrothread.start();
        }

     //   bucketthread = new BucketThread();

     //   bucketthread.start();



        if (motorBackRight != null)
            motorBackRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        if (motorBackLeft != null)
            motorBackLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public RSRobot(GyroSensor gyro) {
        this.gyro = gyro;
    }


    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void SetFrontRightMotor(DcMotor motor) {
        motorFrontRight = motor;
    }

    public void SetFrontLeftMotor(DcMotor motor) {
        motorFrontLeft = motor;
    }

    public void SetBackRightMotor(DcMotor motor) {
        motorBackRight = motor;
    }

    public void SetBackLeftMotor(DcMotor motor) {
        motorBackLeft = motor;
    }

    public void SetHarvesterMotor(DcMotor motor) {
        motorHarvester = motor;
    }

    public void SetSpinnerMotor(DcMotor motor) {motorSpinner = motor;}

    public void setMotorControllerFrontDrive(DcMotorController motorControllerFrontDrive) {
        this.motorControllerFrontDrive = motorControllerFrontDrive;
    }

    public void setMotorControllerRearDrive(DcMotorController motorControllerRearDrive) {
        this.motorControllerRearDrive = motorControllerRearDrive;
    }
/*
    public long DriveForwardLegacy(double power, long distance) throws InterruptedException {
        double encoderTarget;
        double calculatedPow = 0;
        int frontRightStartPosition = 0;
        int currentPosition;
        //set current heading to zero
         gyrothread.setCurrentHeading(0);
        //use a while loop to keep motors going until desired heading reached

        encoderTarget = distance / onemotorclick;

        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        while (motorControllerRearDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
            opMode.telemetry.addData("WAITING FOR READ_ONLY ", 1234);
            opMode.waitForNextHardwareCycle();

        }

        //gets front rights position to be able to zero out the current position
        frontRightStartPosition = motorBackRight.getCurrentPosition();

      //  opMode.telemetry.addData("Current Encoder Position ", frontRightStartPosition);

        currentPosition = frontRightStartPosition;



        while (currentPosition - frontRightStartPosition < encoderTarget) {

            //calculatedPow = calculateTurnPow(degrees, gyrothread.getCurrentheading(), power);
            calculatedPow = power;
            motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

            while (motorControllerRearDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
                opMode.telemetry.addData("WAITING FOR WRITE_ONLY", 1);
                opMode.waitForNextHardwareCycle();
            }


            if (motorControllerRearDrive.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.WRITE_ONLY) {
                motorBackRight.setPower(calculatedPow);


                //motorBackRight.setPower(calculatedPow);
                motorBackLeft.setPower(calculatedPow);

              //  motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);

            }

            //motorBackLeft.setPower(calculatedPow);

            motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
            while (motorControllerRearDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
                opMode.telemetry.addData("WAITING FOR READ_ONLY", 2);
                opMode.waitForNextHardwareCycle();
            }

            currentPosition = motorBackLeft.getCurrentPosition();


            opMode.telemetry.addData("Current Encoder Position ", currentPosition);
            opMode.telemetry.addData("Current Power ", motorBackLeft.getPower());
        }

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        while (motorControllerRearDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
            opMode.telemetry.addData("WAITING FOR WRITE_ONLY", 1);
            opMode.waitForNextHardwareCycle();
        }


        motorBackRight.setPower(0);


        //motorBackRight.setPower(calculatedPow);
        motorBackLeft.setPower(0);


        return (distance);
    }
    */

    private long Spin(double power, long degrees, double direction) throws InterruptedException {
        double calculatedPow = 0;
        //set current heading to zero
        gyrothread.setCurrentHeading(0);
        //use a while loop to keep motors going until desired heading reached
        while (Math.abs(gyrothread.getCurrentHeading()) < (degrees-10)) {
            //calculatedPow = (calculateTurnPow(degrees, gyrothread.getCurrentHeading(), power))*direction*myAlliance.alliance;
            calculatedPow = power*direction*myAlliance.alliance;
            motorFrontRight.setPower(-calculatedPow);
            motorBackRight.setPower(-calculatedPow);
            motorFrontLeft.setPower(calculatedPow);
            motorBackLeft.setPower(calculatedPow);

            opMode.telemetry.addData("curr heading ", gyrothread.getCurrentHeading());
            opMode.telemetry.addData("pow ", calculatedPow);

            opMode.waitForNextHardwareCycle();

        }
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        opMode.waitForNextHardwareCycle();
        return (long) Math.abs(gyrothread.getCurrentHeading());
    }

    public long SpinRight(double power, long degrees) throws InterruptedException {
        //Calling spin function and direction 1 is right
        return(Spin(power, degrees, 1));
    }
    public long SpinLeft(double power, long degrees) throws InterruptedException {
        //Calling spin function and direction -1 is left
        return(Spin(power, degrees, -1));

    }

    double calculateTurnPow(double totalTurn, double currentHeading, double maxPow) {
        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minSpinRampUpPow = .6;
        double minSpinRampDownPow = .6;
        double rampDownCalcPow = 0;
        //number of degrees for speeding up
        double rampUpDegrees = 30;
        //number of degrees for slowing down
        double rampDownDegrees = 30;

        rampUpCalcPow = minSpinRampUpPow + (((maxPow - minSpinRampUpPow) / rampUpDegrees) * currentHeading);
        rampDownCalcPow = minSpinRampDownPow + (((minSpinRampDownPow - maxPow) / rampDownDegrees) * (currentHeading - totalTurn));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);

        if (calculatedPow < minSpinRampDownPow) {
            calculatedPow = minSpinRampDownPow;

        }
        return Range.clip(calculatedPow, -1, 1);
    }

    double calculateDrivePow(double totalDistance, double currentDistance, double maxPow) {
        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minPow = .6;
        double rampDownCalcPow = 0;
        //distance in cm for speeding up
        double rampUpDistance = 20;
        //distance in cm for slowing down
        double rampDownDistance = 20;

        rampUpCalcPow = minPow + (((maxPow - minPow) / rampUpDistance) * currentDistance);
        rampDownCalcPow = minPow + (((minPow - maxPow) / rampDownDistance) * (currentDistance - totalDistance));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);

        if (calculatedPow < minPow) {
            calculatedPow = minPow;

        }
        return Range.clip(calculatedPow, -1, 1);
    }

    private long DriveStallDetection(double power, long distance, double direction) throws InterruptedException {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;
        int leftStallPos = 0;
        int rightStallPos = 0;
        int leftMotorPos=0;
        int rightMotorPos=0;
        int leftStallCutoff;
        int rightStallCutoff;
        int leftStallCount = 0;
        int rightStallCount = 0;
        int leftMagicNumberofDeath = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        int rightMagicNumberofDeath = 4; //(2560 * 0.01)/4;  //encoder ticks per 10msec
        boolean isLeftMotorStalled = false;
        boolean isRightMotorStalled = false;


        //set current heading to zero
        gyrothread.setCurrentHeading(0);
        //use a while loop to keep motors going until desired heading reached

        motorControllerFrontDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        while(motorFrontRight.getCurrentPosition() != 0 || motorBackRight.getCurrentPosition() != 0)
        {
            opMode.waitForNextHardwareCycle();
        }
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        while (Math.abs(motorFrontLeft.getCurrentPosition()) < encoderTarget &&
                Math.abs(motorFrontRight.getCurrentPosition()) < encoderTarget &&
                !isLeftMotorStalled &&
                !isRightMotorStalled) {

            currentHeading = gyrothread.getCurrentHeading();
            //calculatedPow = calculateDrivePow(distance, motorFrontLeft.getCurrentPosition()*onemotorclick, power)*direction;
            leftCalculatedPow = Range.clip((power * direction) - (currentHeading / 10), -1, 1);;
            rightCalculatedPow = Range.clip((power * direction) + (currentHeading / 10), -1, 1);;

            motorBackLeft.setPower(leftCalculatedPow);
            motorBackRight.setPower(rightCalculatedPow);
            motorFrontLeft.setPower(leftCalculatedPow);
            motorFrontRight.setPower(rightCalculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorFrontRight.getCurrentPosition());
            opMode.telemetry.addData("Current Heading ", currentHeading);
            opMode.telemetry.addData("left power ", leftCalculatedPow);
            opMode.telemetry.addData("right power ", rightCalculatedPow);

            leftMotorPos = motorFrontLeft.getCurrentPosition();
            rightMotorPos = motorFrontRight.getCurrentPosition();
            leftStallCutoff = leftMagicNumberofDeath;
            rightStallCutoff = rightMagicNumberofDeath;

//			nxtDisplayBigTextLine( 5, "L %d", leftMotorPos );
            //		nxtDisplayTextLine( 7, "R %d", rightMotorPos );

            //if trying to move and not successfully moving
            if (( leftCalculatedPow != 0) && (Math.abs(leftMotorPos - leftStallPos) <= leftStallCutoff))
            {
                //if stalling for 50 rounds through the loop (.5 second)
                if (++leftStallCount == 20)
                {
                    //left motor has stalled.
                    isLeftMotorStalled = true;

                }
            }
            else
            {
                // not stalled, reset stall counter
                leftStallCount = 0;

                isLeftMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            leftStallPos = leftMotorPos;

            //if trying to move and not successfully moving
            if (( rightCalculatedPow != 0) && (Math.abs(rightMotorPos - rightStallPos) <= rightStallCutoff))
            {
                //if stalling for 50 rounds through the loop
                if (++rightStallCount == 20)
                {
                    //right motor has stalled.
                    isRightMotorStalled = true;


                }
            }
            else
            {
                // not stalled, reset stall counter
                rightStallCount = 0;

                isRightMotorStalled = false;
            }
            //remembers encoder Pos for the next time through the loop
            rightStallPos = rightMotorPos;

            opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();

        if(isLeftMotorStalled || isRightMotorStalled)
        {
            if(leftMotorPos >rightMotorPos)

                distance = (long)(leftMotorPos*onemotorclick);
            else
                distance = (long)(rightMotorPos*onemotorclick);
        }

        return (distance);
    }

    private long Drive(double power, long distance, double direction) throws InterruptedException {
        double encoderTarget;
        double calculatedPow = 0;
        double currentHeading = 0;
        double leftCalculatedPow = 0;
        double rightCalculatedPow = 0;

        //set current heading to zero
        gyrothread.setCurrentHeading(0);
        //use a while loop to keep motors going until desired heading reached

        motorControllerFrontDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
       // motorFrontLeftt.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        while(motorFrontRight.getCurrentPosition() != 0 /*|| motorBackRight.getCurrentPosition() != 0*/)
        {
            opMode.waitForNextHardwareCycle();
        }
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        while (Math.abs(motorFrontRight.getCurrentPosition()) < encoderTarget) {

            currentHeading = gyrothread.getCurrentHeading();
            //calculatedPow = calculateDrivePow(distance, motorFrontLeft.getCurrentPosition()*onemotorclick, power)*direction;
            leftCalculatedPow = Range.clip((power * direction) - (currentHeading / 10), -1, 1);;
            rightCalculatedPow = Range.clip((power * direction) + (currentHeading / 10), -1, 1);;

            motorBackLeft.setPower(leftCalculatedPow);
            motorBackRight.setPower(rightCalculatedPow);
            motorFrontLeft.setPower(leftCalculatedPow);
            motorFrontRight.setPower(rightCalculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorFrontRight.getCurrentPosition());
            opMode.telemetry.addData("Current Heading ", currentHeading);
            opMode.telemetry.addData("left power ", leftCalculatedPow);
            opMode.telemetry.addData("right power ", rightCalculatedPow);
            opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        opMode.waitForNextHardwareCycle();

        return (distance);
    }

    public long DriveForward(double power, long distance) throws InterruptedException {
        //Calling drive function and 1 is forward
        return(Drive(power, distance, 1));
    }
    public long DriveBackward(double power, long distance) throws InterruptedException {
        //Calling drive function and -1 is backward
        return(Drive(power, distance, -1));
    }
    public long DriveForwardStallDetection(double power, long distance) throws InterruptedException {
        //Calling drive function and 1 is forward
        return(DriveStallDetection(power, distance, 1));
    }
    public long DriveBackwardStallDetection(double power, long distance) throws InterruptedException {
        //Calling drive function and -1 is backward
        return(DriveStallDetection(power, distance, -1));
    }
    public void auto_1_OMC(Alliance alliance, long delay) throws InterruptedException
    {
        //allows us to use the code on red and blue
        setMyAlliance(alliance);

        //wait for the specified delay
        opMode.sleep(delay * 1000);



        //robot.SpinRight(1, 360);
        DriveForward(1.0, 85);

        opMode.sleep(1000);

        SpinRight(1, 135);

        opMode.sleep(1000);

        DriveForward(1.0, 300);
    }


    public void auto_3_OMF(Alliance alliance, long delay) throws InterruptedException
    {
        //allows us to use the code on red and blue
        setMyAlliance(alliance);

        //wait for the specified delay
        opMode.sleep(delay * 1000);



        //robot.SpinRight(1, 360);
        DriveForward(1.0, 20);

        opMode.sleep(1500);

        SpinRight(1, 45);

        opMode.sleep(1000);

        DriveForward(1.0, 175);

        opMode.sleep(1000);

        SpinRight(1, 87);

        opMode.sleep(1000);

        DriveForward(1, 300);
    }

    public void auto_1_OMF(Alliance alliance, long delay) throws InterruptedException
    {
        //allows us to use the code on red and blue
        setMyAlliance(alliance);

        //wait for the specified delay
        opMode.sleep(delay * 1000);

        DriveForward(1.0, 85);

        opMode.sleep(1000);

        SpinRight(1, 45);

        opMode.sleep(1000);

        DriveForward(1.0, 60);

        opMode.sleep(1000);

        SpinRight(1, 90);

        opMode.sleep(1000);

        DriveForward(1, 300);
    }

    public void auto_1_PZ(Alliance alliance, long delay) throws InterruptedException
    {
        long distanceDriven = 0;
        setMyAlliance(alliance);

        opMode.sleep(delay * 1000);

        DriveBackward(.5,30);
        opMode.sleep(1000);

        DriveForward(.5,25);

        //start moving
        dropHarvester();

        opMode.sleep(3000);

        recalibrateHarvester();

        reverseSpinner();

        distanceDriven = DriveBackwardStallDetection(.5, 165);
       // DriveBackward(.5, 170);
        if(distanceDriven < 165)
        {
            stopSpinner();
            raiseHarvester();
            DriveForward(.5, 30);
            dropHarvester();
            opMode.sleep(3000);
            reverseSpinner();
            DriveBackward(.5, 165-distanceDriven + 30);
        }


        stopSpinner();

        raiseHarvester();

        opMode.sleep(3000);

        DriveBackward(.5, 10);
    }

    public void auto_3_PZ(Alliance alliance, long delay) throws InterruptedException
    {
        long distanceDriven = 0;
        setMyAlliance(alliance);

        opMode.sleep(delay * 1000);

        //start moving
        dropHarvester();

        opMode.sleep(3000);

        recalibrateHarvester();

        reverseSpinner();

       // distanceDriven = DriveBackwardStallDetection(.5, 330);
         DriveBackward(.5, 210);
        //if(distanceDriven < 330)
        //{
        //    stopSpinner();
        //    raiseHarvester();
        //    DriveForward(.5, 30);
        //    dropHarvester();
        //    opMode.sleep(3000);
        //    reverseSpinner();
        //    DriveBackward(.5, 330-distanceDriven + 30);
        //}


        stopSpinner();

        raiseHarvester();

        opMode.sleep(3000);

        DriveBackward(.5, 75);
    }

    public void auto_3_OMC(Alliance alliance, long delay) throws InterruptedException
    {
        //allows us to use the code on red and blue
        setMyAlliance(alliance);

        //wait for the specified delay
        opMode.sleep(delay * 1000);

        dropHarvester();

        opMode.sleep(3000);

        recalibrateHarvester();

        reverseSpinner();


        //robot.SpinRight(1, 360);
        DriveBackward(1, 45);

        opMode.sleep(1500);

        SpinRight(1, 90);

        opMode.sleep(1000);

        DriveBackward(1, 120);

        opMode.sleep(1000);

        stopSpinner();

        raiseHarvester();

        opMode.sleep(3000);

        SpinLeft(1, 135);

        opMode.sleep (1000);

        DriveForward(1, 150);


    }

    public void initializeHarvester () throws InterruptedException
    {
        bucketThread = new BucketThread(motorHarvester);

        bucketThread.InitializeBucket();

    }

    public void dropHarvester () throws InterruptedException
    {
        bucketThread.targetPos = -1450;
    }

    public void raiseHarvester () throws InterruptedException
    {
        bucketThread.targetPos = -50;
    }

    public void recalibrateHarvester () throws InterruptedException
    {
        //resets the target position to where the bucket is currently (used at ground)
        bucketThread.targetPos = motorHarvester.getCurrentPosition();
    }

    public void reverseSpinner () throws InterruptedException
    {
        motorSpinner.setPower(-1);
    }

    public void intakeSpinner () throws InterruptedException
    {
        motorSpinner.setPower (1);
    }

    public void stopSpinner () throws InterruptedException
    {
        motorSpinner.setPower(0);
    }

    public void startHarvester () throws  InterruptedException
    {

        motorHarvester.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        bucketThread.start();
    }

    public void stopHarvester () throws  InterruptedException
    {
        bucketThread.stopBucketThread();
    }


    public void pre_Match_Test() throws InterruptedException
    {
        opMode.sleep(1000);

        DriveForward(1, 100);

        opMode.sleep(1000);

        DriveBackward(1, 100);

        opMode.sleep(1000);

        SpinRight(1, 90);

        opMode.sleep(1000);

        SpinLeft(1, 90);

        //move servoZip

        //spin collector

        //move collection bucket

        //move scoring bucket

        //move scoring bucket servo

        //move climber dumper

        //move hanger thingy

        //move button presser
    }



}