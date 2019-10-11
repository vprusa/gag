package cz.gag.visualization;

import cz.gag.common.Hand;

public class HandVisualization extends HandVisualizationBase {

    public FingerVisualization thumpVis = new FingerVisualization(hi, 50, 50, 50, 50);
    public FingerVisualization indexVis = new FingerVisualization(hi, 25, 112, 60, 45, 30);
    public FingerVisualization middleVis = new FingerVisualization(hi, 0, 120, 70, 50, 35);
    public FingerVisualization ringVis = new FingerVisualization(hi, -25, 112, 65, 48, 30);
    public FingerVisualization littleVis = new FingerVisualization(hi, -50, 102, 45, 35, 25);

    public HandVisualization(Hand hi) {
        super(hi);
    }

    public void draw() {
        // thumpVis.rotateX(0.3f);
        thumpVis.draw();

        // indexVis.rotateZ(-0.5f);
        indexVis.draw();

        // no rotation in Y ;)
        // middleVis.rotateY(-0.5f);
        middleVis.draw();

        // ringVis.drawStaticPart();
        // ringVis.drawParts();
        ringVis.draw();

        // littleVis.drawStaticPart();
        // littleVis.drawParts();
        littleVis.draw();
    }

    public void rotate(float angle, float rotationX, float rotationY, float rotationZ) {
        super.rotate(angle, rotationX, rotationY, rotationZ);

        // translate magic
        app.pushMatrix();
        app.translate(0, 0, 0);
        /*
         * app.rotateX(rotationX * (hi.ordinal() == 0 ? 1f : -1f));
         * app.rotateY(rotationY * (hi.ordinal() == 0 ? 1f : -1f));
         * app.rotateZ(rotationZ * (hi.ordinal() == 0 ? 1f : -1f));
         */
        float hiv = (hi.ordinal() == 0 ? 1f : -1f);
        app.rotate(angle, rotationX * hiv, rotationY * hiv, rotationZ * hiv);

        draw();
        app.translate(0, 0, 0);
        app.popMatrix();
    }
}
