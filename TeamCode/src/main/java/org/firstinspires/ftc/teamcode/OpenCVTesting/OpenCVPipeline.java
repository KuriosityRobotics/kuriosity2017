package org.firstinspires.ftc.teamcode.OpenCVTesting;

import android.app.Activity;
import android.content.Context;

import org.opencv.android.CameraBridgeViewBase.*;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Mat;

/**
 * Created by guinea on 6/19/17.
 * This is a base class for an OpenCV pipeline loop.
 * TODO: consider more functionality in here, relating to camera parameter manipulation? For most cases it's not needed.
 */

public abstract class OpenCVPipeline implements CvCameraViewListener2 {
//    static {
//        System.loadLibrary("opencv_java3");
//    }
    private JavaCameraView cameraView;
    private Context context;
    private boolean initStarted = false;
    private boolean inited = false;


    public void init(Context context) {
        init(context, 0);
    }

//    0 is back, 1 is front
    public void init(Context context, final int cameraIndex) {
        this.initStarted = true;
        this.context = context;
        final Activity activity = (Activity) context;
        final Context finalContext = context;
        final CvCameraViewListener2 self = this;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                // JCVs must be instantiated on a UI thread
                cameraView = new JavaCameraView(finalContext, cameraIndex);
                cameraView.setCameraIndex(cameraIndex);
                cameraView.setCvCameraViewListener(self);
                inited = true;
            }
        });
    }
    //TODO: allow other JavaCameraViews to be used, with the stipulation that they must already be instantiated from a UI thread

    /**
     * Attaches the underlying JavaCameraView to the screen and the camera using the set to do so, essentially starting OpenCV processing.
     * This function must be called after init().
     * @throws IllegalStateException if enable() is called before init()
     */
    public void enable() {
        if (!initStarted) throw new IllegalStateException("init() needs to be called before an OpenCVPipeline can be enabled!");
        // this is an absolute hack
        try {
            while (!inited) Thread.sleep(10);
        } catch (InterruptedException e) { return; }

        cameraView.enableView();
    }

    /**
     * Detaches the JavaCameraView from the camera and the screen, stopping OpenCV processing.
     * Be careful not to:
     *     - disable twice
     *     - disable before enabling
     * because dean kamen help you if something bad happens from that
     */
    public void disable() {
        cameraView.disableView();
    }

    /**
     * Exposes the underlying JavaCameraView used. Before init() is called, this is null.
     * @return the JavaCameraView.
     */
    public JavaCameraView getCameraView() {
        return cameraView;
    }

    /**
     * This function is called when the camera is started; overriding this may be useful to set the
     * maximum width and height parameters of an image processing pipeline.
     * @param width -  the width of the frames that will be delivered
     * @param height - the height of the frames that will be delivered
     */
    @Override
    public void onCameraViewStarted(int width, int height) {
        // override this if you wish
    }

    /**
     * Override this function if there should be logic on camera close.
     */
    @Override
    public void onCameraViewStopped() {

    }

    /**
     * The method that calls {@link #processFrame(Mat, Mat)}; there's little reason to override this, if ever.
     * @param inputFrame the input frame given by the internal JavaCameraView
     * @return the result of {@link #processFrame(Mat, Mat)}
     */
    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        return processFrame(inputFrame.rgba(), inputFrame.gray());
    }

    /**
     * Override this with the main image processing logic. This is run every time the camera recieves a frame.
     * @param rgba a {@link Mat} that is in RGBA format
     * @param gray a {@link Mat} that is already grayscale
     * @return the Mat that should be displayed to the screen; in most cases one would probably just want to return rgba
     */
    public abstract Mat processFrame(Mat rgba, Mat gray);
}
