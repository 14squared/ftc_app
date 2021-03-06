package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * class is instantiated on the Robot Controller and executed.
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RedAutonomousOp", group="Linear Opmode")

//@Disabled
public class AsherRed2 extends LinearOpMode {
    private InvadersRelicRecoveryBot homeCode = null;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        homeCode = new InvadersRelicRecoveryBot( );
        homeCode.init(this);
        homeCode.setJewelArmPosition(0, InvadersRelicRecoveryBot.JewelPush.Left);
        homeCode.setJewelArmPosition(1, InvadersRelicRecoveryBot.JewelPush.Right);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        homeCode.leftGrab.setPosition(0.3);
        homeCode.rightGrab.setPosition(0.3);

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {

            runtime.reset();

            /* Spin To Win: */
            //1. Lower Jewel Arm
            homeCode.setJewelArmPosition(0.22, InvadersRelicRecoveryBot.JewelPush.Right);
            homeCode.sleepMs(3000);

            //2. Read Jewel Color
            boolean iSawRed = homeCode.isRightJewelRed();
            boolean iSawBlue = homeCode.isRightJewelBlue();

            // 3.If Red move arm this way:
            if(iSawRed)
            {
                //@todo wiggle the robot this way
                homeCode.encoderDrive( 0.60, 4, 4, 2);
                homeCode.sleepMs(1000);
                homeCode.stopMotors();
                homeCode.setJewelArmPosition(1, InvadersRelicRecoveryBot.JewelPush.Right);
                homeCode.sleepMs(2000);
                homeCode.encoderDrive(1, 8, 8, 5);
                homeCode.stopMotors();
            }
            //Else if blue move arm that way.
            else if(iSawBlue)
            {
                //@todo wiggle the robot the other way
                homeCode.encoderDrive( 0.60, -4, -4, 2);
                homeCode.sleepMs(1000);
                homeCode.stopMotors();
                homeCode.setJewelArmPosition(1, InvadersRelicRecoveryBot.JewelPush.Right);
                homeCode.sleepMs(2000);
                homeCode.encoderDrive(1, 16, 16, 5);
                homeCode.stopMotors();
            }
            else
            {
                homeCode.setJewelArmPosition(1, InvadersRelicRecoveryBot.JewelPush.Right);
                homeCode.sleepMs(2000);
                homeCode.encoderDrive(1, 12, 12, 5);
                homeCode.stopMotors();
            }
            //4. Raise Jewel Arm
            //   5. Drive to VuMark
            //@todo Decide whether to drive to the VuMark
//
//            //   6. Read VuMark
//            RelicRecoveryVuMark vuMark = homeCode.getVuforiaTargets(false);
//
//            //   7. Calculate Cryptobox distance with VuMark = X.
//            //   8. Drive Straight. Distance = X.
//            if(vuMark == RelicRecoveryVuMark.LEFT)
//            {
//                //Drive to the left
//                homeCode.setDriveTrainPower(0.5);
//            }else if(vuMark == RelicRecoveryVuMark.RIGHT)
//            {
//                homeCode.setDriveTrainPower(0.3);
//
//            }else if(vuMark == RelicRecoveryVuMark.CENTER){
//                //Criss Cross Criss Cross
//                homeCode.setDriveTrainPower(0.2);
//            }else{
//                //Everybody clap your hands
//                homeCode.setDriveTrainPower(0.1);
//            }
//
//
//
//
//            //   9. Turn to Cryptobox
//            homeCode.encoderDrive(0.2,36,36,9);
//
//            //   10. Put block in
//
//            //   11. Drive straight into triangle.
//            //   12. Block Enemy Targets
//
//
//            // Send calculated power to wheels



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
         //   break;
        //}


    }
}