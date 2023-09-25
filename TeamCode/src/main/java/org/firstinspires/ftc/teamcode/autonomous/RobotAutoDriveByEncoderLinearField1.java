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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="FIELD1-AutoDriveByEncoder", group="Robot")
//@Disabled
public class RobotAutoDriveByEncoderLinearField1 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor lfDrive = null;

    private DcMotor rfDrive = null;

    private DcMotor rbDrive = null;

    private DcMotor lbDrive = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: GOBUILDA Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        lfDrive  = hardwareMap.get(DcMotor.class, "lf_motor");
        rfDrive = hardwareMap.get(DcMotor.class, "rf_motor");
        rbDrive  = hardwareMap.get(DcMotor.class, "rb_motor");
        lbDrive  = hardwareMap.get(DcMotor.class, "lb_motor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);
        rbDrive.setDirection(DcMotor.Direction.FORWARD);

        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          lfDrive.getCurrentPosition(),
                          rfDrive.getCurrentPosition(),
                          lbDrive.getCurrentPosition(),
                          rbDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // Team Note: DRIVE_SPEED IS FOR FORWARD AND BACK MOVEMENT.
        // Positive numbers are Forward and Negative numbers are Backward

        // Team Note: TURN_SPEED IS FOR TURNING THE ROBOT LEFT AND RIGHT
        // Positive numbers are for Left and Negative numbers are for Right

        // Team Note: to strafe, the two wheels in the direction you want to go turn inwards and the other wheels turn outwards
        // to strafe right, use DRIVE_SPEED, lfInches and rbInches are positive, and lbInches and rfInches are negative.
        // to strafe left, use DRIVE_SPEED, lfInches and rbInches are negative, and lbInches and rfInches are positive.

        // we want the robot to go forward 3 feet. turn right 90 degrees. and go forward 8 feet

        encoderDrive(DRIVE_SPEED, 36, 36, 36, 36, 5.0);
        encoderDrive(TURN_SPEED, -12, -12, -12, -12, 4.0);
        encoderDrive(DRIVE_SPEED, 96, 96, 96, 96, 4.0);

        encoderDrive(DRIVE_SPEED, -36, 36, 36, -36, 5.0);
        encoderDrive(DRIVE_SPEED, 96, 96, 96, 96, 4.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
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
                             double lfInches, double rfInches,
                             double lbInches, double rbInches,
                             double timeoutS) {
        int newLfTarget;
        int newRfTarget;
        int newLbTarget;
        int newRbTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLfTarget = lfDrive.getCurrentPosition() + (int)(lfInches * COUNTS_PER_INCH);
            newRfTarget = rfDrive.getCurrentPosition() + (int)(rfInches * COUNTS_PER_INCH);
            newLbTarget = lbDrive.getCurrentPosition() + (int)(lbInches * COUNTS_PER_INCH);
            newRbTarget = rbDrive.getCurrentPosition() + (int)(rbInches * COUNTS_PER_INCH);
            lfDrive.setTargetPosition(newLfTarget);
            rfDrive.setTargetPosition(newRfTarget);
            lbDrive.setTargetPosition(newLbTarget);
            rbDrive.setTargetPosition(newRbTarget);

            // Turn On RUN_TO_POSITION
            lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lfDrive.setPower(Math.abs(speed));
            rfDrive.setPower(Math.abs(speed));
            lbDrive.setPower(Math.abs(speed));
            rbDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (lfDrive.isBusy() && rfDrive.isBusy() && lbDrive.isBusy() && rbDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLfTarget,  newRfTarget, newLbTarget,  newRbTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            lfDrive.getCurrentPosition(), rfDrive.getCurrentPosition(),
                                            lbDrive.getCurrentPosition(), rbDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lfDrive.setPower(0);
            rfDrive.setPower(0);
            lbDrive.setPower(0);
            rbDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(60000);   // optional pause after each move.
        }
    }
}
