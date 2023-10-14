package org.firstinspires.ftc.teamcode.TeamProp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "TeamPropAutonomous")
public class TeamPropAuton extends OpMode {
    private DcMotor lf_drive;
    private DcMotor rf_drive;
    private DcMotor lb_drive;
    private DcMotor rb_drive;
    private OpenCvCamera camera;
    private TeamPropDetect teamPropPipeline;

    // Constants for encoder counts and movement speed
    private static final double COUNTS_PER_REVOLUTION = 537.7; // Adjust for your motor
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // Adjust for your wheels
    private static final double DRIVE_SPEED = 0.6;

    @Override
    public void init() {
        // Initialize motors (configure motor names as per your robot configuration)
        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        lb_drive = hardwareMap.get(DcMotor.class, "lb_drive");
        rb_drive = hardwareMap.get(DcMotor.class, "rb_drive");

        // Reverse motors if needed
        // lf_drive.setDirection(DcMotor.Direction.REVERSE);
        // rf_drive.setDirection(DcMotor.Direction.FORWARD);
        // lb_drive.setDirection(DcMotor.Direction.REVERSE);
        // rb_drive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize motor run modes
        lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Specify the camera name from your robot configuration
        String cameraName = "Technoverse Cam"; // Replace with the actual camera name

        // Initialize the camera hardware
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cameraName));

        // Set the active camera pipeline
        teamPropPipeline = new TeamPropDetect();
        camera.setPipeline(teamPropPipeline);
    }

    @Override
    public void start() {
        // Configure camera settings (e.g., resolution and rotation)
        camera.openCameraDevice();
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void loop() {
        // If the team prop is detected, navigate to it
        if (teamPropPipeline.isObjectDetected()) {
            // Calculate the distance to move based on the detected X-coordinate
            double teamPropX = teamPropPipeline.getObjectLocation().x;
            double distanceToMove = calculateDistanceToMove(teamPropX);

            // Move to the team prop
            if (distanceToMove > 0) {
                driveForward(distanceToMove);
            }

            // Perform additional actions or tasks if needed
        }
    }

    // Calculate the distance to move based on the detected X-coordinate
    private double calculateDistanceToMove(double targetX) {
        // Constants for camera and robot configuration
        double targetXCenter = 160; // The X-coordinate at the center of the camera frame
        double maxSpeed = 0.5; // Maximum speed for the robot
        double proportionalConstant = 0.01; // Proportional control constant (adjust as needed)

        // Calculate the error, which is the difference between the detected X-coordinate and the center
        double error = targetXCenter - targetX;

        // Calculate the motor power adjustment based on the error using proportional control
        double powerAdjustment = error * proportionalConstant;

        // Limit the power adjustment to stay within the maximum speed
        powerAdjustment = Math.max(-maxSpeed, Math.min(maxSpeed, powerAdjustment));

        // Calculate the distance to move based on the power adjustment
        // The assumption here is that a non-zero power adjustment means there's an error in positioning
        // You can further adjust this logic based on your robot's behavior
        return (powerAdjustment != 0) ? 10.0 : 0.0; // 10 inches is an arbitrary distance
    }



    // Drive forward for a specified distance (in inches)
    private void driveForward(double inches) {
        int targetPosition = (int) (COUNTS_PER_REVOLUTION * inches / (WHEEL_DIAMETER_INCHES * Math.PI));

        // Set the target position for all four motors
        lf_drive.setTargetPosition(targetPosition);
        rf_drive.setTargetPosition(targetPosition);
        lb_drive.setTargetPosition(targetPosition);
        rb_drive.setTargetPosition(targetPosition);

        // Set the drive speed
        lf_drive.setPower(DRIVE_SPEED);
        rf_drive.setPower(DRIVE_SPEED);
        lb_drive.setPower(DRIVE_SPEED);
        rb_drive.setPower(DRIVE_SPEED);

        // Set run-to-position mode
        lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait until motors reach the target position
        while (lf_drive.isBusy() || rf_drive.isBusy() || lb_drive.isBusy() || rb_drive.isBusy()) {
            // You can add additional tasks or checks here
        }

        // Stop the motors
        lf_drive.setPower(0);
        rf_drive.setPower(0);
        lb_drive.setPower(0);
        rb_drive.setPower(0);

        // Set the motors back to the original run mode
        lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
