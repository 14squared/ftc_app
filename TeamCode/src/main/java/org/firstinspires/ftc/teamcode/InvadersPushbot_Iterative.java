/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This file provides Telop driving for the IfSpace Invaders 2015/16 Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * Most device access is managed through the HardwarePushbot class.  A touch sensor was added to
 * limit our robot arm range of motion and is manually connected to the robot during init().
 *
 * This particular OpMode provides single-thumb navigation of PushBot using the left-thumbstick.
 * It raises and lowers the claw using the right-thumbstick.
 * It also opens and closes the claws slowly using the left and right Trigger buttons.
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



//This opmode lets the driver drive with only one stick. This means that we have a lot of buttons to map to other things.

@TeleOp(name="Bigly", group="Pushbot")
//@Disabled
public class InvadersPushbot_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    // Will be connected to PushBot's Limit Switch
    TouchSensor limitSwitch;                         // Will be connected to PushBot's Limit Switch

    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    GyroSensor gyroSensor;

    OpticalDistanceSensor distanceSensor;

    Gamepad lastGamePadState = new Gamepad();




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Connect our limit switch TouchSensor object to the Robot
        limitSwitch = hardwareMap.touchSensor.get("down limit");
        assert (limitSwitch != null);

        // Send telemetry message to signify robot waiting;
         updateTelemetry(telemetry);
        colorSensor1 = hardwareMap.colorSensor.get("color1");
        colorSensor2 = hardwareMap.colorSensor.get("color2");
        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        colorSensor1.enableLed(false);
        colorSensor2.enableLed(false);
        distanceSensor = hardwareMap.opticalDistanceSensor.get("ODS");
        gyroSensor.calibrate();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Use the left joystick to move the robot forwards/backwards and turn left/right
        double x = gamepad1.left_stick_x; // Note: The joystick goes negative when pushed forwards, so negate it
        double y = -gamepad1.left_stick_y; // Note: The joystick goes negative when pushed right, so negate it

        // Algorithm for setting power to left/right motors based on joystick x/y values
        // note: The Range.clip function just ensures we stay between Â±100%
        left = Range.clip(y-x, -1, +1);
        right = Range.clip(y+x, -1, +1);

        // Call the setPower functions with our calculated values to activate the motors
        robot.frontLeft.setPower(left);
        robot.frontRight.setPower(right);
        robot.backRight.setPower(right);
        robot.backLeft.setPower(left);

        // Read our limit switch to see if the arm is too high
        boolean limitTriggered = limitSwitch.isPressed();

        if(limitTriggered) {
            robot.BallElevator.setPower(0);  //Elevator off
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("switch", "%s", limitTriggered ? "Triggered" : "Open");
        telemetry.addData("Pusher", robot.pusher.getPosition());
        telemetry.addData("beacon", robot.beacon.getPosition());
        telemetry.addData("Color1: ", "R%d,G%d,B%d", colorSensor1.red(), colorSensor1.green(), colorSensor1.blue());
        telemetry.addData("Color2: ", "R%d,G%d,B%d", colorSensor2.red(), colorSensor2.green(), colorSensor2.blue());
        telemetry.addData("Distance:", "Light%.2f", distanceSensor.getLightDetected());
        telemetry.addData("RawGyro:", "X%d, Y%d, Z%d", gyroSensor.rawX(), gyroSensor.rawY(), gyroSensor.rawZ());
        telemetry.addData("GyroHeading:", "Heading%d", gyroSensor.getHeading());

        updateTelemetry(telemetry);

        //Beacon button and pusher button

        robot.beacon.setPosition(1-gamepad1.left_trigger);
        robot.pusher.setPosition(1-(gamepad1.right_trigger*0.5));  // Limit pusher range from 100% to 50% (ie all the way open to halfway closed)

        if (gamepad1.a == true){
            if (limitSwitch.isPressed() == true){
                setBallElevator( 0);
            }
            else {

                setBallElevator(-1); // Elevator down
            }
        }
        else if (gamepad1.y == true){
            //robot.BallElevator.setPower(1);
            setBallElevator(1);
        }
        else {
            setBallElevator(0);
        }

        if (gamepad1.start == true){
            //robot.LeftBallLauncher.setPower(-1);
            //robot.RightBallLauncher.setPower(-1);
            setLauncherPower(1);
        }
        else if (gamepad1.back == true) {
            //robot.LeftBallLauncher.setPower(0);
            //robot.RightBallLauncher.setPower(0);
            setLauncherPower(0);
            if(!limitTriggered) {
                setBallElevator(-1);  //Elevator down
            }
        }
    }

    void setBallElevator(float power)
    {
        //@todo Write to a file what we're about to do to the motor here
        robot.BallElevator.setPower(power);
    }

    void setLauncherPower(float power){
        robot.LeftBallLauncher.setPower(-power);
        robot.RightBallLauncher.setPower(-power);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.pusher.setPosition(0.5);
        robot.beacon.setPosition(0.1);
        robot.BallElevator.setPower(0);
        robot.frontRight.setPower(0.0);
        robot.frontLeft.setPower(0.0);
        robot.backRight.setPower(0.0);
        robot.backLeft.setPower(0.0);
        robot.LeftBallLauncher.setPower(0.0);
        robot.RightBallLauncher.setPower(0.0);
    }
}
