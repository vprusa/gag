package cz.gag.visualization;

import cz.gag.common.Configuration;
import cz.gag.common.Hand;
import cz.gag.common.ProcessingApplet;

public class HandVisualizationBase {
    public float rotationX = 0.0f;
    public float rotationY = 0.0f;
    public float rotationZ = 0.0f;
    public float angle = 0.0f;

    protected ProcessingApplet app;
    protected Hand hi;

    public HandVisualizationBase(Hand hi) {
        this.app = Configuration.app;
        this.hi = hi;
    }

    public void rotate(float angle, float rotationX, float rotationY, float rotationZ) {
        this.angle = angle;
        this.rotationX = rotationX;
        this.rotationY = rotationY;
        this.rotationZ = rotationZ;
    }

    public void rotateX(float rotationX) {
        this.rotationX = rotationX;
    }

    public void rotateY(float rotationY) {
        this.rotationY = rotationY;
    }

    public void rotateZ(float rotationZ) {
        this.rotationZ = rotationZ;
    }

    public void rotateAngle(float angle) {
        this.angle = angle;
    }
}
