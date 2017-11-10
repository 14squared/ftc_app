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


    /* Matthew, Willow, Alyssa - I just changed the 'period' variable below from private to public.
       This means that you can see this variable from your opModes (e.g. robot.period.reset()).
       This is a useful object for calculating how much time has elapsed (in milliseconds)
       Usage Example:
       // 1st: Define a variable for how many milliseconds you want to do something
       int maxTimeInMilliSecondsIwantMyFunctionToTake = 5000; // 5000mS = 5 seconds

       // 2nd: Reset the elapsed timer with the reset() method
       period.reset(); // Call reset to 'start' the timer

       // 3rd: Do a bunch of work inside a while loop()
       // Here is an example that shows how you could try to see a blue beacon while driving and then
       // stop either when you found it, or when your timer expired.
          boolean iSawBlue = false;

          // Start Driving Forwards
          leftDrive.setPower(1);
          leftDrive.setPower(1);
          while(period.time() < maxTimeInMilliSecondsIwantMyFunctionToTake) {
             // Keep checking the soIseeBlueLeft() function inside this while loop to look for the beacon
             if(soIseeBlueLeft()) {
                 // Hooray: We found the beacon!  Set our boolean variable to true!
                 iSawBlue = true;
                 break; // This 'break' will halt the while loop.
          }
          // Stop driving.  We either saw blue, or we timed out after 5 seconds.
          leftDrive.setPower(0);
          rightDrive.setPower(0);

          if(iSawBlue == true) {
             // Do something neat if we saw blue (maybe activate our beacon pusher (or drive again until we see red)
          }
     */
    public ElapsedTime period  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



    // Add the new Motors/Servos from the .txt file into the Relic



    /* Public OpMode members. */

    public DcMotor leftDrive   = null;
    public DcMotor rightDrive  = null;
    public Servo leftGrab = null;
    public Servo rightGrab = null;
    public Servo jewelPushLeft = null;
    public Servo jewelPushRight = null;
    public DcMotor liftMotor = null;
    public CRServo CRGripper = null;


    public ModernRoboticsI2cRangeSensor UDSLeft = null;   // Default I2C Address: 0x26
    public ModernRoboticsI2cRangeSensor UDSRight = null;  // Default I2C Address: 0x28

    public ColorSensor jewelSensorLeft = null;
    public FtcI2cDeviceState beaconSensorLeftState;

    public ColorSensor jewelSensorRight = null;
    public FtcI2cDeviceState beaconSensorRightState;

    public ColorSensor floorSensor = null;
    public FtcI2cDeviceState floorSensorState;

    public ModernRoboticsI2cGyro gyro = null;
    public FtcI2cDeviceState gyroState;

    public TouchSensor touchSensor = null;

//FUNCTIONS
    public boolean isRightJewelRed(){
        ///@todo Add Color Sensing here when we create the color sensor
        boolean jewelColorRed = false;
        if(jewelSensorRight.red() > 200)
        {

            jewelColorRed = true;
            //Move robot arm right
        }
        return jewelColorRed;
    }


    public boolean isRightJewelBlue(){
        ///@todo Add Color Sensing here when we create the color sensor
        boolean jewelColorBlue = false;
    if(jewelSensorRight.blue() > 200){
        jewelColorBlue = true;
    }
        return false;
    }public boolean isLeftJewelRed(){
    ///@todo Add Color Sensing here when we create the color sensor
    return false;
}
    public boolean isLeftJewelBlue(){
        ///@todo Add Color Sensing here when we create the color sensor
        return false;
    }

    public enum JewelPush {Left, Right}

    public void setJewelArmPosition(double servoPosition, JewelPush robotSide)
    {
        telemetry.addData(
                "setJewelArmPosition",
                "pos: %.02f, side: %s",
                servoPosition,
                robotSide == JewelPush.Right ? "RIGHT" : "LEFT");
        if(robotSide == JewelPush.Left)
        {
            //Sets left jewel arm position
            jewelPushLeft.setPosition(servoPosition);
        }
        else
        {
            //Set right jewel arm position
            jewelPushRight.setPosition(servoPosition);
        }
    }


    /**
     * GyroTurn function allows our robot to do make precise +/- degrees turns using the gyro sensor
     //* @param speed currently unused.
     //* @param degrees indicates the number of degrees to turn left or right.  Positive numbers
     *                will turn the robot right the specified degrees, negative numbers will turn the
     *                robot left the specified number of degrees.
     *
     */
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
        //@todo getMaxSpeed and setMaxSpeed were removed from ftc_app SDK v3.0
        //int oldMaxSpeedRight = rightDrive.getMaxSpeed();
        //int oldMaxSpeedLeft = leftDrive.getMaxSpeed();
        //leftDrive.setMaxSpeed(250);
        //rightDrive.setMaxSpeed(250);

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
            //Thread.yield();
            //sleepMs(3000);

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
            //Thread.yield();
            //sleepMs(3000);
        }
        rightDrive.setPower(0);
        leftDrive.setPower(0);
        //@todo getMaxSpeed and setMaxSpeed were removed from ftc_app SDK v3.0
        //leftDrive.setMaxSpeed(oldMaxSpeedLeft);
        //rightDrive.setMaxSpeed(oldMaxSpeedRight);
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


//// This function is currently disabled.
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;}


        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            moveCounts = (int)(distance * COUNTS_PER_INCH);
//            newLeftTarget = leftDrive.getCurrentPosition() + moveCounts;
//            newRightTarget = rightDrive.getCurrentPosition() + moveCounts;
//
//            // Set Target and Turn On RUN_TO_POSITION
//            leftDrive.setTargetPosition(newLeftTarget);
//            rightDrive.setTargetPosition(newRightTarget);
//
//            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // start motion.
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            leftDrive.setPower(speed);
//            rightDrive.setPower(speed);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (leftDrive.isBusy() && rightDrive.isBusy())) {
//
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = speed - steer;
//                rightSpeed = speed + steer;
//
//                // Normalize speeds if any one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                leftDrive.setPower(leftSpeed);
//                rightDrive.setPower(rightSpeed);
//
//                // Display drive status for the driver.
//                //telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//                //telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//                //telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
//                        rightDrive.getCurrentPosition();
//                //telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
//                //telemetry.update();
//            }
//
//            // Stop all motion;
//            leftDrive.setPower(0);
//            rightDrive.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }

    public boolean doIseeBlueLeft () {
        return (jewelSensorLeft.blue() >= 5);
    }

    public boolean doIseeRedLeft () {
        return (jewelSensorLeft.red() >= 5);
    }

    public boolean doIseeBlueRight () {
        return (jewelSensorRight.blue() >= 5);
    }

    public boolean doIseeRedRight () {
        return (jewelSensorRight.red() >= 5);
    }

    public void turnToAbsoluteHeading(double speed, double absoluteHeading, int timeoutMs) {
        simpleGyroTurn(speed,absoluteHeading-gyro.getIntegratedZValue(),timeoutMs);
    }

    // Right/Clockwise = Positive Turn Degrees
    // Left/CounterClockwise = Negative Turn Degrees
    // timeoutMs provides a sanity check to make sure we don't turn forever
    public void simpleGyroTurn(double speed, double turnDegrees, int timeoutMs)
    {
        // Our turn by degrees algorithm has a bit of slop in it.
        // Try to correct the user input to match reality
        // If turnDegrees == 100, then reset to 90 (which makes our robot turn 100)
        //turnDegrees *= .90;

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

    static double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static double     DRIVE_GEAR_REDUCTION    = 1.0;      // This is < 1.0 if geared UP
    static double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
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
    public void init(OpMode activeOpMode){
        // Save reference to Hardware map
        hwMap = activeOpMode.hardwareMap;

        // Save reference to the OpMode's Telemetry
        telemetry = activeOpMode.telemetry;

        // Save reference to the active OpMode
        this.activeOpMode = activeOpMode;

        try {
            // Define and Initialize Drive Motors
            leftDrive = hwMap.get(DcMotor.class, "leftDrive");
            rightDrive = hwMap.get(DcMotor.class, "rightDrive");

            telemetry.addData("Gear Reduction (L)", "%.02f", leftDrive.getMotorType().getGearing());
            telemetry.addData("Encoder PPR (L)", "%.02f", leftDrive.getMotorType().getTicksPerRev());

            telemetry.addData("Gear Reduction (R)", "%.02f", rightDrive.getMotorType().getGearing());
            telemetry.addData("Encoder PPR (R)", "%.02f", rightDrive.getMotorType().getTicksPerRev());

            COUNTS_PER_MOTOR_REV = leftDrive.getMotorType().getTicksPerRev();
            DRIVE_GEAR_REDUCTION = leftDrive.getMotorType().getGearing();
            COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
        }


        catch (IllegalArgumentException e)
        {
            // If we have a robot (e.g. SmallBot) that doesn't have all of the motors defined, then
            // we throw an exception and can't test the other, installed Motors.  This try/catch block swallows that exception.
            telemetry.addData("'leftDrive' or 'rightDrive' not defined in config", e);
        }

        try {
            liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        }
        catch (IllegalArgumentException e)
        {
            // If we have a robot (e.g. SmallBot) that doesn't have all of the motors defined, then
            // we throw an exception and can't test the other, installed Motors.  This try/catch block swallows that exception.
            telemetry.addData("'liftMotor' not defined in config", e);
        }

        try {
            // Define installed servos
            leftGrab = hwMap.get(Servo.class, "leftGrab");
            rightGrab = hwMap.get(Servo.class, "rightGrab");
        }
        catch (IllegalArgumentException e)
        {
            // If we have a robot (e.g. SmallBot) that doesn't have all of the motors defined, then
            // we throw an exception and can't test the other, installed Motors.  This try/catch block swallows that exception.
            telemetry.addData("'leftGrab' or 'rightGrab' not defined in config", e);
        }

        try {
            jewelPushLeft = hwMap.get(Servo.class, "jewelPushLeft");
            jewelPushRight = hwMap.get(Servo.class, "jewelPushRight");
            CRGripper = hwMap.get(CRServo.class, "CRGripper");
            CRGripper.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch (IllegalArgumentException e)
        {
            // If we have a robot (e.g. SmallBot) that doesn't have all of the motors defined, then
            // we throw an exception and can't test the other, installed Motors.  This try/catch block swallows that exception.
            telemetry.addData("'jewelPushLeft' or 'jewelPushRight' not defined in config'", e);
        }

        try {
            // Define our sensors
            jewelSensorLeft = hwMap.colorSensor.get("jewelSensorLeft");
            jewelSensorRight = hwMap.colorSensor.get("jewelSensorRight");
        }
        catch (IllegalArgumentException e)
        {
            // If we have a robot (e.g. SmallBot) that doesn't have all of the motors defined, then
            // we throw an exception and can't test the other, installed Motors.  This try/catch block swallows that exception.
            telemetry.addData("'jewelSensorLeft' or 'jewelSensorRight' not defined in config", e);
        }

            // These sensors are leftovers from the VelocityVortex game - we may add these kinds
            // of sensors to our RelicRecovery bot in the future.  Leaving for reference.
                //touchSensor = hwMap.touchSensor.get("downLimit");
                //UDSLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "UDSLeft");
                //UDSRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "UDSRight");
                //floorSensor = hwMap.colorSensor.get("floorSensor");
                //gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyroSensor");

        if(leftDrive != null) leftDrive.setDirection(DcMotor.Direction.REVERSE);
        if(rightDrive != null) rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setDriveTrainPower(0);
        if(liftMotor != null)
        {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0);
        }

        // Set all non-driving motors to run without encoders.
        if(leftDrive != null) {
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(rightDrive != null) {
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        // Custom I2C Addresses Go Here!
        ///@todo Need to validate that UDSLeft.setI2cAddress is working with new SDK.  Hasn't been tested since VelocityVortex championship
        if(UDSLeft != null) UDSLeft.setI2cAddress(I2cAddr.create8bit(0x26));
        if(floorSensor != null) floorSensor.setI2cAddress(I2cAddr.create8bit(0x3A));
        if(jewelSensorRight != null) jewelSensorRight.setI2cAddress(I2cAddr.create8bit(0x36));

        // Initialize Color Sensor LEDs to off
        if(jewelSensorLeft != null) jewelSensorLeft.enableLed(false);
        if(jewelSensorRight != null) jewelSensorRight.enableLed(false);
        if(floorSensor != null) floorSensor.enableLed(false);

        // If we have a gyro on our robot, stop for a moment to calibrate and reset to 0
        if(gyro != null) {
            gyro.calibrate();

            // make sure the gyro is calibrated before continuing
            while (gyro.isCalibrating()) {
                //sleepMs(50);
            }
            gyro.resetZAxisIntegrator();
        }
    }

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

    /*
 *  Method to perform a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 */
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

    public RelicRecoveryVuMark getVuforiaTargets(boolean UseFrontCam){
        VuforiaLocalizer vuforia;
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
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
    enum TurnDirection {Left, Right}

    //Move in a straight line forward or backwards
    //speed is positive for forward, negative for backwards
    //speed can be between -1.0 and 1.0
    void moveStraight(double speed){
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    private void tankTurn(double speed, TurnDirection direction){
        if (direction == TurnDirection.Left){
            leftDrive.setPower(-speed);
            rightDrive.setPower(speed);
        } else {
            leftDrive.setPower(speed);
            rightDrive.setPower(-speed);
        }
    }

    private void forwardTurn(double speed, TurnDirection direction){
        if (direction == TurnDirection.Left){
            leftDrive.setPower(speed/2);
            rightDrive.setPower(speed);
        } else {
            leftDrive.setPower(speed);
            rightDrive.setPower(speed/2);
        }
    }

    private void stopMotors(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
