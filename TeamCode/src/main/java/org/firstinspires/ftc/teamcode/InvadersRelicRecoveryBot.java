package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is the IfSpace Invaders 2017/2018 Relic Recovery season robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Ref: .\IfSpaceInvaders_ModernRobotics_Hardware_Defintions.txt for details
 */
public class InvadersRelicRecoveryBot
{
    public static final double MID_SERVO       =  0.5 ;
    /* local OpMode members. */
    HardwareMap hwMap           = null;
    Telemetry telemetry         = null;
    OpMode activeOpMode         = null;



    public ElapsedTime period  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /* Public OpMode members. */

    public DcMotor leftDrive   = null;
    public DcMotor rightDrive  = null;
    public Servo leftGrab = null;
    public Servo rightGrab = null;
    //public Servo jewelPush = null;
    public DcMotor liftMotor = null;


    public Servo   beaconLeft  = null;
    public Servo   beaconRight  = null;

    public ModernRoboticsI2cRangeSensor UDSLeft = null;   // Default I2C Address: 0x26
    public ModernRoboticsI2cRangeSensor UDSRight = null;  // Default I2C Address: 0x28

    public ColorSensor beaconSensorLeft = null;
    public FtcI2cDeviceState beaconSensorLeftState;

    public ColorSensor beaconSensorRight = null;
    public FtcI2cDeviceState beaconSensorRightState;

    public ColorSensor floorSensor = null;
    public FtcI2cDeviceState floorSensorState;

    public ModernRoboticsI2cGyro gyro = null;
    public FtcI2cDeviceState gyroState;

    public TouchSensor touchSensor = null;


    public void init(OpMode activeOpMode) {
        // Save reference to Hardware map
        hwMap = activeOpMode.hardwareMap;

        // Save reference to the OpMode's Telemetry
        telemetry = activeOpMode.telemetry;

        // Save reference to the active OpMode
        this.activeOpMode = activeOpMode;


        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive = hwMap.get(DcMotor.class, "rightDrive");
        leftGrab = hwMap.get(Servo.class, "leftGrab");
        rightGrab = hwMap.get(Servo.class, "rightGrab");
        //jewelPush = hwMap.get(Servo.class, "jewelPush");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        // Set all non-driving motors to run without encoders.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


//FUNCTIONS

    /**
     * GyroTurn function allows our robot to do make precise +/- degrees turns using the gyro sensor
     //* @param speed currently unused.
     //* @param degrees indicates the number of degrees to turn left or right.  Positive numbers
     *                will turn the robot right the specified degrees, negative numbers will turn the
     *                robot left the specified number of degrees.
     *
     */
    public boolean isJewelRed(){
        ///@todo Add Color Sensing here when we create the color sensor
        return false;
    }
    public boolean isJewelBlue(){
        ///@todo Add Color Sensing here when we create the color sensor
        return false;
    }

    public void GyroTurn(double speed, double degrees,int timeoutMs) {
        simpleGyroTurn(speed,degrees,timeoutMs);
    }


    public void DriveToWall(float distance, DistanceUnit distanceUnit, double power, int timeoutMs) {
        period.reset();
        setDriveTrainPower(power);
        while ((period.time() < timeoutMs) && opModeIsActive()) {
            if(UDSLeft.getDistance(distanceUnit) <= distance) break;
            if(UDSRight.getDistance(distanceUnit) <= distance) break;
        }
        setDriveTrainPower(0);
    }

    private boolean opModeIsActive(){
        boolean isActive = true;
        // If we're running a linear op mode, then make sure we stop when our opmode is no longer active
        if(activeOpMode instanceof LinearOpMode)
        {
            isActive = ((LinearOpMode)activeOpMode).opModeIsActive();
        }
        return isActive;
    }

    public void AlignToWall(double DistanceFromTarget, DistanceUnit unit){
        double rightDistance = UDSRight.getDistance(unit);
        double leftDistance = UDSLeft.getDistance(unit);
        int timeoutMs = 3000; // Limit this function to a maximum of 3s
        //@TODO: 11/4/2017 Figure out what the replacements for get and set maxspeed are. They were removed in version 3 of the SDK.

        if(leftDistance > rightDistance == true){
            telemetry.addData("ATW TURNING", "RIGHT");
            telemetry.addData("RANGE: ","R: %.02f, L: %.02f", rightDistance, leftDistance);
            telemetry.update();
            //Thread.yield();
            //sleepMs(3000);

            period.reset();
            leftDrive.setPower(1);
            rightDrive.setPower(-1);
            while(opModeIsActive() && (period.time() <= timeoutMs) && (leftDistance > rightDistance)){
                rightDistance = UDSRight.getDistance(unit);
                leftDistance = UDSLeft.getDistance(unit);
                telemetry.addData("RANGE: ","R: %.02f, L: %.02f", rightDistance, leftDistance);
                telemetry.update();
            }
        }
        else if (leftDistance < rightDistance == true){
            telemetry.addData("ATW TURNING", "LEFT");
            telemetry.addData("RANGE: ","R: %.02f, L: %.02f", rightDistance, leftDistance);
            telemetry.update();


            period.reset();
            leftDrive.setPower(-1);
            rightDrive.setPower(1);
            while(opModeIsActive() && (period.time() <= timeoutMs) && (leftDistance < rightDistance)){
                rightDistance = UDSRight.getDistance(unit);
                leftDistance = UDSLeft.getDistance(unit);
                telemetry.addData("RANGE: ","R: %.02f, L: %.02f", rightDistance, leftDistance);
                telemetry.update();
            }
        }
        else if (leftDistance == rightDistance){
            // Do Nothing - We are aligned to the wall
            telemetry.addData("TURNING", "-----");
            telemetry.addData("RANGE: ","R: %.02f, L: %.02f", rightDistance, leftDistance);
            telemetry.update();

        }
        rightDrive.setPower(0);
        leftDrive.setPower(0);
    }

    public void setDriveTrainPower(double power)
    {
        setDriveTrainPower(power,power);
    }

    public void setDriveTrainPower(double leftPower, double rightPower)
    {
        if(leftDrive != null) leftDrive.setPower(leftPower);
        if(rightDrive != null) rightDrive.setPower(rightPower);
    }



    public void turnToAbsoluteHeading(double speed, double absoluteHeading, int timeoutMs) {
        simpleGyroTurn(speed,absoluteHeading-gyro.getIntegratedZValue(),timeoutMs);
    }

    public void simpleGyroTurn(double speed, double turnDegrees, int timeoutMs)
    {
        int currentHeading = gyro.getIntegratedZValue();
        double targetHeading = currentHeading - turnDegrees;
        // Early Return (do nothing) if we are giving a zero degree turn
        if(turnDegrees == 0) return;
        // Start our timeout-detection timer
        period.reset();

        if(turnDegrees > 0) {
            setDriveTrainPower(speed,-speed);
            while ((period.time() < timeoutMs) && opModeIsActive()) {
                currentHeading = gyro.getIntegratedZValue();
                // Exit our while loop because we're at our destination
                if((currentHeading - targetHeading) <= 0) break;
            }
        }
        else {
            setDriveTrainPower(-speed,speed);
            while ((period.time() < timeoutMs) && opModeIsActive()) {
                currentHeading = gyro.getIntegratedZValue();
                // Exit our while loop because we're at our destination
                if((targetHeading - currentHeading) <= 0) break;
            }
        }
        setDriveTrainPower(0);
    }

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            //telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            //telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



    public void DriveToWhiteLine(double power, int intensity, boolean enableLed, int timeoutMs){
        setDriveTrainPower(power);
        floorSensor.enableLed(enableLed);
        period.reset();
        while (opModeIsActive()){
            int currentIntensity = floorSensor.alpha();
            telemetry.addData("Alpha: ", "%03d", currentIntensity);

            // Break out of the loop if we found the line
            if(currentIntensity >= intensity) break;

            // Break out of the loop if we have timed out
            if(period.time() > timeoutMs) break;
        }

        setDriveTrainPower(0);
        floorSensor.enableLed(false);
    }

    /* Constructor */
    public InvadersRelicRecoveryBot(){

    }

    public void sleepMs(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
        }
    }


    /* Initialize standard Hardware interfaces */


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    //Cold pizza normally.
    public enum CapBallState {
        UP,
        DOWN,
        OFF
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            setDriveTrainPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (period.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            setDriveTrainPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleepMs(250);   // optional pause after each move
        }
    }

    /*
 *  Method to perfmorm a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 */
    public void timedDrive(double speed, double durationMs) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            setDriveTrainPower(speed);
            period.reset();
            while((period.time() < durationMs) && opModeIsActive()) {
            }
            // Stop all motion;
            setDriveTrainPower(0);

            //sleepMs(250);
        }
    }

    public RelicRecoveryVuMark GetVuforiaTargets(boolean UseFrontCam){
        VuforiaLocalizer vuforia;
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AeLUXiL/////AAAAGYerUEv7r0F7nofgcRK24JQpfTb4xU+veUOD5pc/T5znab3eZ685JtMtKTR9fDT7en0PyNfozvli09GISBHVm7J/k6go7TmM9d4Shx7gjAFpPI0d/56kfPA8g7PtWdubIEMN66TY6iQKPN4sQBJXv6pNa+w2ThxOrTJwn4dvo5rgBG9HGm+WyD/ZwMo/f6dKgG20pvukjc2+dtbrUsPKAuE6MChQ2xPRISjtvfQ18Ajg6Gcy6A4c0zXLMehjewwhNRTxL4eYYAAUZzboVhA/o/Peh5/5LZIT9le1hlBBoT7WhzwAtAO54CFV0M80TYO66pOdTZqAsAYtvUUgmU3ZANb5/0PRK9346UZu4Ty1uhgU";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        if(UseFrontCam = true) {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        }
        else {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        }
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary




        relicTrackables.activate();



        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);


        switch(vuMark){
            case LEFT:
                telemetry.addData("VuMark LEFT", "is visible", vuMark);
                break;
            case CENTER:
                telemetry.addData("VuMark CENTER", "is visible", vuMark);
                break;
            case RIGHT:
                telemetry.addData("VuMark RIGHT", "is visible", vuMark);
                break;
            case UNKNOWN:
                telemetry.addData("VuMark", "Not Found", vuMark);
                break;
        }


        telemetry.update();
        return vuMark;

    }
}
