/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import cz.gag.common.ProcessingApplet;

public class Button {
    int x;
    int y;
    int w;
    int h;
    String label = "";
    public boolean overRegion = false;
    public boolean pressed = false;
    public boolean button = false;
    
    public Button(String label, int x, int y, int w, int h) {
        super();
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
        this.label = label;
    }

    public void draw() {
        if(Configuration.app == null) {return;}
        Configuration.app.rect(x, y, w, h);
        Configuration.app.fill(255, 0, 0);
        Configuration.app.text(label, x + w / 2 - label.length() * 3f, y + h / 2 + 5);
    }

    public boolean isIn(int X, int Y) {
        if(Configuration.app == null) {return false;}
        int xc = x + (Configuration.app.width / 2);
        int yc = y + (Configuration.app.height / 2);

        if (Y > yc && Y < yc + h && X > xc && X < xc + w)
            return true;
        // print("X: " + X + " Y: " + Y + "\n");
        // print("x: " + x + " y: " + y + "\n");
        // print("xc: " + xc + " yc: " + yc + "\n");
        return false;
    }
}

