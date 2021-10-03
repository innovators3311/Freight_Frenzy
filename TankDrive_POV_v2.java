/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tank Drive POV Controlling v2", group="Linear Opmode")
public class TankDrive_POV_v2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;
    //declare here all custom variables needed before the game starts
    boolean toggling_buttons[] = {false, false, false};
    // 0- gamepad.a 1- gamepad.left_bumper 2-gampepad.right_bumper  Don't forget to make sure that the right item of the array is used!
    boolean powerIsLocked = false;
    double lockedPower[] = {0, 0};


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightDrive = hardwareMap.get(DcMotor.class, "rf");
        leftDriveBack  = hardwareMap.get(DcMotor.class, "lb");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rb");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            if (toggling_buttons[0] && ! gamepad1.a) {
                toggling_buttons[0] = false;
                if (powerIsLocked == false) {
                    powerIsLocked = true;
                    lockedPower[0] = leftPower;
                    lockedPower[1] = rightPower;
                } else {
                    powerIsLocked = false;
                }
            }

            if (toggling_buttons[1] && ! gamepad1.left_bumper) {
                toggling_buttons[1] = false;
                lockedPower[0] = Range.clip(lockedPower[0] + 0.1, -1.0, 1.0);
            }

            if (toggling_buttons[2] && ! gamepad1.right_bumper) {
                toggling_buttons[2] = false;
                lockedPower[1] = Range.clip(lockedPower[1] + 0.1, -1.0, 1.0);
            }

            // Send calculated power to wheels
            if (powerIsLocked) {
                leftDrive.setPower(lockedPower[0]);
                rightDrive.setPower(lockedPower[1]);
                leftDriveBack.setPower(lockedPower[0]);
                rightDriveBack.setPower(lockedPower[1]);
            } else {
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
                leftDriveBack.setPower(leftPower);
                rightDriveBack.setPower(rightPower);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Power locked", powerIsLocked);
            telemetry.update();

            //flip correct boolean value if the corresponding key is pressed
            if (gamepad1.a) {
                toggleValues(0);
            }
            if (gamepad1.left_bumper) {
                toggleValues(1);
            }
            if (gamepad1.right_bumper) {
                toggleValues(2);
            }
        }
    }
    //declaring toggle function
    private void toggleValues(int button_number) {
        toggling_buttons[button_number] = true;
    }
}
