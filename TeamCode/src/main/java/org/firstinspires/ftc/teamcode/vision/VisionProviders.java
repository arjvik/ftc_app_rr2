package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.opencv.OpenCVIntegration;

public final class VisionProviders {
    private VisionProviders() { throw new RuntimeException("Utility Class"); }

    public static final Class<? extends VisionProvider>[] visionProviders =
            new Class[]{OpenCVIntegration.class, DummyVisionIntegration.class};


    public static final Class<? extends VisionProvider> defaultProvider = DummyVisionIntegration.class;

}
