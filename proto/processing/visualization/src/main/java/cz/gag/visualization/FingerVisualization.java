package cz.gag.visualization;

import cz.gag.common.Hand;

public class FingerVisualization extends HandVisualizationBase {
    // TODO change to finger parts or smth.. maybe?
    public int sideOffset;
    public int length1;
    public int length2;
    public int length3;
    public int length4; // if == -1 then is thumb

    public int length2offset;
    public int length3offset;
    public int length4offset; // if == -1 then is thumb

    public int offsetY = 0; // TODO add to constructors offset[X,Y,Z]

    // for thumb
    public FingerVisualization(Hand hi, int sideOffset, int length1, int length2offset, int length3offset) {
        this(hi, sideOffset, length1, length2offset, length3offset, -1);
    }

    public FingerVisualization(Hand hi, int sideOffset, int length1, int length2offset, int length3offset,
                               int length4offset) {
        super(hi);
        this.sideOffset = sideOffset;
        this.length2offset = length2offset;
        this.length3offset = length3offset;
        this.length4offset = length4offset;
        recalcLenghts(length1, length2offset, length3offset, length4offset);
    }

    void recalcLenghts(int length1, int length2offset, int length3offset, int length4offset) {
        this.length1 = length1;
        this.length2 = this.length1 + length2offset;
        this.length3 = this.length2 + length3offset;
        if (length4offset != -1) {
            this.length4 = this.length3 + length4offset;
        } else {
            this.length4 = -1;
        }
    }

    public void drawStaticPart() {
        app.lineWithDot(0, offsetY, 0, sideOffset, length1, 0);
    }

    public void drawParts() {
        // drawStaticPart();
        // app.lineWithDot(fingerStartX, fingerStartY, 0, sideOffset, this.length1, 0);
        app.lineWithDot(sideOffset, this.length1, 0, sideOffset, this.length2, 0);
        if (this.length4 == -1 || this.length4offset == -1) {
            this.app.strokeWeight(1);
            this.app.line(sideOffset, this.length2, 0, sideOffset, this.length3, 0);
        } else {
            app.lineWithDot(sideOffset, this.length2, 0, sideOffset, this.length3, 0);
            this.app.strokeWeight(1);
            this.app.line(sideOffset, this.length3, 0, sideOffset, this.length4, 0);
        }
    }

    public void draw() {
        int length1bkp = this.length1;
        int sideOffsetbkp = this.sideOffset;
        drawStaticPart();
        // translate magic
        app.pushMatrix();
        app.translate(0, 0, 0);
        app.translate(sideOffset, this.length1, 0);
        this.length1 = 0;
        recalcLenghts(this.length1, length2offset, length3offset, length4offset);

        /*
         * app.rotateX(rotationX * (hi.ordinal() == 0 ? 1f : -1f));
         * app.rotateY(rotationY * (hi.ordinal() == 0 ? 1f : -1f));
         * app.rotateZ(rotationZ);// * (hi.ordinal() == 0 ? 1f : -1f));
         */

        float hiv = (hi.ordinal() == 0 ? 1f : -1f);
        app.rotate(angle, rotationX * hiv, rotationY * hiv, rotationZ * hiv);
        app.stroke(0, 255, 0);
        app.point(0, 0, 0);
        sideOffset = 0;
        drawParts();
        app.translate(0, 0, 0);
        app.popMatrix();
        app.stroke(255, 0, 255); // purple
        this.length1 = length1bkp;
        this.sideOffset = sideOffsetbkp;
    }
}
