package org.firstinspires.ftc.teamcode.vision.opencv;

import org.opencv.core.Mat;

public abstract class OpenCVColorFilter {
    public abstract void process(Mat input, Mat mask);
}