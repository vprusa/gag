/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

class Line3D {
    Point3D begin, end;

    Line3D(Point3D b, Point3D e) {
        begin = b;
        end = e;
    }

    void draw() {
        if(Configuration.app == null) {return;}
        Configuration.app.line(begin.x, begin.y, begin.z, end.x, end.y, end.z);
    }
}
