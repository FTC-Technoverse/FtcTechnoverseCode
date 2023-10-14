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

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.5;
    private static final long DRIVE_TIME = 10000;
    private static final double TARGET_X = 160; // Adjust the target X-coordinate as needed
    private static final double X_THRESHOLD = 10; // Adjust the threshold for X-coordinate detection

    private long startTime;

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
        startTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();
        long elapsedTime = currentTime - startTime;

        if (elapsedTime < DRIVE_TIME) {
            if (teamPropPipeline.isObjectDetected()) {
                double teamPropX = teamPropPipeline.getObjectLocation().x;

                if (Math.abs(teamPropX - TARGET_X) > X_THRESHOLD) {
                    // Turn the robot to align with the team prop
                    if (teamPropX < TARGET_X) {
                        // Turn left
                        lf_drive.setPower(-TURN_SPEED);
                        rf_drive.setPower(TURN_SPEED);
                        lb_drive.setPower(-TURN_SPEED);
                        rb_drive.setPower(TURN_SPEED);
                    } else {
                        // Turn right
                        lf_drive.setPower(TURN_SPEED);
                        rf_drive.setPower(-TURN_SPEED);
                        lb_drive.setPower(TURN_SPEED);
                        rb_drive.setPower(-TURN_SPEED);
                    }
                } else {
                    // Drive forward
                    lf_drive.setPower(DRIVE_SPEED);
                    rf_drive.setPower(DRIVE_SPEED);
                    lb_drive.setPower(DRIVE_SPEED);
                    rb_drive.setPower(DRIVE_SPEED);
                }
            } else {
                // If no team prop is detected, stop the robot
                lf_drive.setPower(0);
                rf_drive.setPower(0);
                lb_drive.setPower(0);
                rb_drive.setPower(0);
            }
        } else {
            // Stop the robot after the specified drive time
            lf_drive.setPower(0);
            rf_drive.setPower(0);
            lb_drive.setPower(0);
            rb_drive.setPower(0);
        }
    }
}
