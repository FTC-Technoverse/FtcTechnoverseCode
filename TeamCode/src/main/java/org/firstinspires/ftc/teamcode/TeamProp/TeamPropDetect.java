package org.firstinspires.ftc.teamcode.TeamProp;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class TeamPropDetect extends OpenCvPipeline {
    private boolean isObjectDetected = false;
    private Point objectLocation = null;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to grayscale
        Mat grayImage = new Mat();
        Imgproc.cvtColor(input, grayImage, Imgproc.COLOR_RGB2GRAY);

        // Apply Gaussian blur for noise reduction
        Imgproc.GaussianBlur(grayImage, grayImage, new Size(5, 5), 0);

        // Detect edges using Canny edge detection
        Mat edges = new Mat();
        Imgproc.Canny(grayImage, edges, 100, 200);

        // Find contours in the edge-detected image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset detection results
        isObjectDetected = false;
        objectLocation = null;

        // Iterate through detected contours
        for (MatOfPoint contour : contours) {
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f newMat = new MatOfPoint2f(contour.toArray());

            // Approximate the contour to a polygon
            Imgproc.approxPolyDP(newMat, approxCurve, 0.02 * Imgproc.arcLength(newMat, true), true);

            // Check if the contour is a square (you can adjust this based on your team prop's shape)
            if (approxCurve.total() == 4) {
                // This is likely the team prop
                Rect boundingRect = Imgproc.boundingRect(contour);
                double cX = boundingRect.x + boundingRect.width / 2.0;
                double cY = boundingRect.y + boundingRect.height / 2.0;

                // Draw a circle at the center of the prop
                Imgproc.circle(input, new Point(cX, cY), 10, new Scalar(0, 255, 0), -1);

                // Store detection results
                isObjectDetected = true;
                objectLocation = new Point(cX, cY);
            }
        }

        return input;
    }

    public boolean isObjectDetected() {
        return isObjectDetected;
    }

    public Point getObjectLocation() {
        return objectLocation;
    }
}
