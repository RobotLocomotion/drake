package drake.util;

import java.io.*;
import java.lang.Math;
import lcm.lcm.LCM;

/**
* A Java LCMGL client
**/
public class BotLCMGLClient {

  ByteArrayOutputStream _bouts = new ByteArrayOutputStream();
  DataOutputStream _outs = new DataOutputStream(_bouts);
  String _channel = "LCMGL";
  LCM _lcm = null;
  String _name = "unnamed";
  int _scene = 0;
  int _sequence = 0;

  public final static int LCMGL_BEGIN         = 4;
  public final static int LCMGL_END           = 5;
  public final static int LCMGL_VERTEX3F      = 6;
  public final static int LCMGL_VERTEX3D      = 7;
  public final static int LCMGL_COLOR3F       = 8;
  public final static int LCMGL_COLOR4F       = 9;
  public final static int LCMGL_POINTSIZE     = 10;
  public final static int LCMGL_ENABLE        = 11;
  public final static int LCMGL_DISABLE       = 12;
  public final static int LCMGL_BOX           = 13;
  public final static int LCMGL_CIRCLE        = 14;
  public final static int LCMGL_LINE_WIDTH    = 15;
  public final static int LCMGL_NOP           = 16;
  public final static int LCMGL_VERTEX2D      = 17;
  public final static int LCMGL_VERTEX2F      = 18;
  public final static int LCMGL_TEXT          = 19;
  public final static int LCMGL_DISK          = 20;
  public final static int LCMGL_TRANSLATED    = 21;
  public final static int LCMGL_ROTATED       = 22;
  public final static int LCMGL_LOAD_IDENTITY = 23;
  public final static int LCMGL_PUSH_MATRIX   = 24;
  public final static int LCMGL_POP_MATRIX    = 25;
  public final static int LCMGL_RECT          = 26;
  public final static int LCMGL_TEXT_LONG     = 27;
  public final static int LCMGL_NORMAL3F      = 28;
  public final static int LCMGL_SCALEF        = 29;
  public final static int LCMGL_MULT_MATRIXF  = 30;
  public final static int LCMGL_MULT_MATRIXD  = 31;
  public final static int LCMGL_MATERIALF     = 32;
  public final static int LCMGL_PUSH_ATTRIB    = 33;
  public final static int LCMGL_POP_ATTRIB     = 34;
  public final static int LCMGL_DEPTH_FUNC     = 35;
  public final static int LCMGL_TEX_2D         = 36;
  public final static int LCMGL_TEX_DRAW_QUAD  = 37;
  public final static int LCMGL_SPHERE         = 38;
  public final static int LCMGL_CYLINDER       = 39;
  public final static int LCMGL_MATRIX_MODE    = 40;
  public final static int LCMGL_ORTHO          = 41;

  public final static int LCMGL_POINTS         = 0x0000;
  public final static int LCMGL_LINES          = 0x0001;
  public final static int LCMGL_LINE_LOOP      = 0x0002;
  public final static int LCMGL_LINE_STRIP     = 0x0003;
  public final static int LCMGL_TRIANGLES      = 0x0004;
  public final static int LCMGL_TRIANGLE_STRIP = 0x0005;
  public final static int LCMGL_TRIANGLE_FAN   = 0x0006;
  public final static int LCMGL_QUADS          = 0x0007;
  public final static int LCMGL_QUAD_STRIP     = 0x0008;
  public final static int LCMGL_POLYGON        = 0x0009;

  public final static int LCMGL_SCALE_TO_VIEWER_AR = 42;

  public BotLCMGLClient(LCM lcm, String name) { 
    _lcm = lcm;
    _name = name;
  }

  private synchronized void add0(int cmd) {
    try { _outs.writeByte(cmd); } catch(IOException xcp) {}
  }

  private synchronized void add1i(int cmd, int arg) {
    try { 
      _outs.writeByte(cmd); 
      _outs.writeInt(arg);
    } catch(IOException xcp) {}
  }

  private synchronized void add1f(int cmd, float arg) {
    try {     
      _outs.writeByte(cmd); 
      _outs.writeFloat(arg);
    } catch(IOException xcp) {}
  }

  private synchronized void add2f(int cmd, float x, float y) {
    try { 
      _outs.writeByte(cmd); 
      _outs.writeFloat(x);
      _outs.writeFloat(y);
    } catch(IOException xcp) {}
  }

  private synchronized void add2d(int cmd, double x, double y) {
    try { 
      _outs.writeByte(cmd); 
      _outs.writeDouble(x);
      _outs.writeDouble(y);
    } catch(IOException xcp) {}
  }

  private synchronized void add3f(int cmd, float x, float y, float z) {
    try { 
      _outs.writeByte(cmd); 
      _outs.writeFloat(x);
      _outs.writeFloat(y);
      _outs.writeFloat(z);
    } catch(IOException xcp) {}
  }

  private synchronized void add3d(int cmd, double x, double y, double z) {
    try { 
      _outs.writeByte(cmd); 
      _outs.writeDouble(x);
      _outs.writeDouble(y);
      _outs.writeDouble(z);
    } catch(IOException xcp) {}
  }

  private synchronized void add4f(int cmd, float x, float y, float z, float w) {
    try { 
      _outs.writeByte(cmd); 
      _outs.writeFloat(x);
      _outs.writeFloat(y);
      _outs.writeFloat(z);
      _outs.writeFloat(w);
    } catch(IOException xcp) {}
  }

  private synchronized void add4d(int cmd, double x, double y, double z, double w) {
    try { 
      _outs.writeByte(cmd); 
      _outs.writeDouble(x);
      _outs.writeDouble(y);
      _outs.writeDouble(z);
      _outs.writeDouble(w);
    } catch(IOException xcp) {}
  }

  public synchronized void switchBuffers() {
    byte[] b = _bouts.toByteArray();
    bot_lcmgl.data_t msg = new bot_lcmgl.data_t();
    msg.name = _name;
    msg.scene = _scene;
    msg.sequence = _sequence;
    msg.data = b;
    msg.datalen = b.length;

    _lcm.publish(_channel, msg);

    _scene++;
    _sequence++;
    _bouts.reset();
  }

  public synchronized void glBegin(int mode) { add1i(LCMGL_BEGIN, mode); }
  public synchronized void glEnd() { add0(LCMGL_END); }

  public synchronized void glEnable(int cap) { add1i(LCMGL_ENABLE, cap); }
  public synchronized void glDisable(int cap) { add1i(LCMGL_DISABLE, cap); }

  public synchronized void glPushMatrix() { add0(LCMGL_PUSH_MATRIX); }
  public synchronized void glPopMatrix() { add0(LCMGL_POP_MATRIX); }
  public synchronized void glLoadIdentity() { add0(LCMGL_LOAD_IDENTITY); }
  
  public synchronized void glMultMatrixf(float f[]) {
    try {
      _outs.writeByte(LCMGL_MULT_MATRIXF);
      for(int i=0; i<16; i++)
        _outs.writeFloat(f[i]);
    } catch(IOException xcp) {}
  }
  public synchronized void glMultMatrixd(double f[]) {
    try {
      _outs.writeByte(LCMGL_MULT_MATRIXF);
      for(int i=0; i<16; i++)
        _outs.writeDouble(f[i]);
    } catch(IOException xcp) {}
  }

  public synchronized void glPointSize(float size) { add1f(LCMGL_POINTSIZE, size); }
  public synchronized void glLineWidth(float size) { add1f(LCMGL_LINE_WIDTH, size); }

  public synchronized void glColor3f(float r, float g, float b) { add3f(LCMGL_COLOR3F, r, g, b); }
  public synchronized void glColor4f(float r, float g, float b, float a) { add4f(LCMGL_COLOR4F, r, g, b, a); }

  public synchronized void glScalef(float x, float y, float z) { add3f(LCMGL_SCALEF, x, y, z); }
  public synchronized void glTranslated(double x, double y, double z) { add3d(LCMGL_TRANSLATED, x, y, z); }
  public synchronized void glRotated(double theta, double x, double y, double z) { add4d(LCMGL_ROTATED, theta, x, y, z); }

  public synchronized void glNormal3f(float x, float y, float z) { add3f(LCMGL_NORMAL3F, x, y, z); }

  public synchronized void glVertex2d(double x, double y) { add2d(LCMGL_VERTEX2D, x, y); }
  public synchronized void glVertex2f(float x, float y) { add2f(LCMGL_VERTEX2F, x, y); }

  public synchronized void glVertex3d(double x, double y, double z) { add3d(LCMGL_VERTEX3D, x, y, z); }
  public synchronized void glVertex3f(float x, float y, float z) { add3f(LCMGL_VERTEX3F, x, y, z); }

  public synchronized void circle(double x, double y, double z, double r) {
    try {
      _outs.writeByte(LCMGL_CIRCLE); 
      _outs.writeDouble(x);
      _outs.writeDouble(y);
      _outs.writeDouble(z);
      _outs.writeFloat((float)r);
    } catch(IOException xcp) {}
  }

  public synchronized void disk(double x, double y, double z, double r_in, double r_out) {
    try {
      _outs.writeByte(LCMGL_DISK); 
      _outs.writeDouble(x);
      _outs.writeDouble(y);
      _outs.writeDouble(z);
      _outs.writeFloat((float)r_in);
      _outs.writeFloat((float)r_out);
    } catch(IOException xcp) {}
  }
  
  public static final int TEXT_DROP_SHADOW = 1;
  public static final int TEXT_JUSTIFY_LEFT = 2;
  public static final int TEXT_JUSTIFY_RIGHT = 4;
  public static final int TEXT_JUSTIFY_CENTER = 8;
  public static final int TEXT_ANCHOR_LEFT = 16;
  public static final int TEXT_ANCHOR_RIGHT = 32;
  public static final int TEXT_ANCHOR_TOP = 64;
  public static final int TEXT_ANCHOR_BOTTOM = 128;
  public static final int TEXT_ANCHOR_HCENTER = 256;
  public static final int TEXT_ANCHOR_VCENTER = 512;
  public static final int TEXT_NORMALIZED_SCREEN_COORDINATES = 1024;
  public static final int TEXT_MONOSPACED = 2048;    
  
  public synchronized void text(double[] xyz, String text) {
    this.text(xyz, text, 0, 
          TEXT_DROP_SHADOW | 
          TEXT_JUSTIFY_CENTER |
          TEXT_ANCHOR_HCENTER |
          TEXT_ANCHOR_VCENTER);
  }
  
  public synchronized void text(double[] xyz, String text, int font, int flags) {
    try {
      _outs.writeByte(LCMGL_TEXT_LONG);
      _outs.writeInt(font);
      _outs.writeInt(flags);
      _outs.writeDouble(xyz[0]);
      _outs.writeDouble(xyz[1]);
      _outs.writeDouble(xyz[2]);
      _outs.writeInt(text.length());
      _outs.writeBytes(text);
    } catch(IOException xcp) {}
  }
  
  public synchronized void glOrtho(double left, double right, double bottom, double top, double nearVal, double farVal) {
    try {
      _outs.writeByte(LCMGL_ORTHO); 
      _outs.writeDouble(left);
      _outs.writeDouble(right);
      _outs.writeDouble(bottom);
      _outs.writeDouble(top);
      _outs.writeDouble(nearVal);
      _outs.writeDouble(farVal);
    } catch(IOException xcp) {}
  }

  public synchronized void glMaterialf(int face, int name, float c0, float c1, float c2,float c3) {
    try {
      _outs.writeByte(LCMGL_MATERIALF); 
      _outs.writeInt(face);
      _outs.writeInt(name);
      _outs.writeFloat(c0);
      _outs.writeFloat(c1);
      _outs.writeFloat(c2);
      _outs.writeFloat(c3);
    } catch(IOException xcp) {}
  }

  public synchronized void glPushAttrib(int attrib) { add1i(LCMGL_PUSH_ATTRIB,attrib); } 
  public synchronized void glPopAttrib() { add0(LCMGL_POP_ATTRIB); }
  public synchronized void glDepthFunc(int func) { add1i(LCMGL_DEPTH_FUNC,func); }

  public synchronized void box(double[] xyz, float[] size) {
    try {
      _outs.writeByte(LCMGL_BOX); 
      _outs.writeDouble(xyz[0]);
      _outs.writeDouble(xyz[1]);
      _outs.writeDouble(xyz[2]);
      _outs.writeFloat(size[0]);
      _outs.writeFloat(size[1]);
      _outs.writeFloat(size[2]);
    } catch(IOException xcp) {}
  }
  
  public synchronized void cylinder(double[] base_xyz, double base_radius, double top_radius, double height, int slices, int stacks) {
    try {
      _outs.writeByte(LCMGL_CYLINDER); 
      _outs.writeDouble(base_xyz[0]);
      _outs.writeDouble(base_xyz[1]);
      _outs.writeDouble(base_xyz[2]);
      _outs.writeDouble(base_radius);
      _outs.writeDouble(top_radius);
      _outs.writeDouble(height);
      _outs.writeInt(slices);
      _outs.writeInt(stacks);
    } catch(IOException xcp) {}
  }

  public synchronized void sphere(double[] xyz, double radius, int slices, int stacks) {
    try {
      _outs.writeByte(LCMGL_SPHERE); 
      _outs.writeDouble(xyz[0]);
      _outs.writeDouble(xyz[1]);
      _outs.writeDouble(xyz[2]);
      _outs.writeDouble(radius);
      _outs.writeInt(slices);
      _outs.writeInt(stacks);
    } catch(IOException xcp) {}
  }

  public synchronized void glDrawAxes() {
    //x-axis
    glBegin(LCMGL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glEnd();
    //y-axis
    glBegin(LCMGL_LINES);
    glColor3f(0, 1, 0);
    glVertex3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glEnd();
    //z-axis
    glBegin(LCMGL_LINES);
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glEnd();
  }

  public synchronized void line(double x_start, double y_start, double x_end, double y_end) {
    glBegin(LCMGL_LINES);
    glVertex2d(x_start, y_start);
    glVertex2d(x_end, y_end);
    glEnd();
  }

  public synchronized void orthoCircles3d() {
    circle(0,0,0,1);
    line(-1, 0, 1, 0);
    line(0, -1, 0, 1);

    glPushMatrix();
    glRotated(90, 1, 0, 0);
    circle(0,0,0,1);
    line(-1, 0, 1, 0);
    line(0, -1, 0, 1);
    glPopMatrix();

    glPushMatrix();
    glRotated(90, 0, 1, 0);
    circle(0,0,0,1);
    line(-1, 0, 1, 0);
    line(0, -1, 0, 1);
    glPopMatrix();
  }

  public synchronized void line3(double x_start, double y_start, double z_start, double x_end, double y_end, double z_end) {
    glBegin(LCMGL_LINES);
    glVertex3d(x_start, y_start, z_start);
    glVertex3d(x_end, y_end, z_end);
    glEnd();
  }

  public synchronized void points(double[] x, double[] y, double[] z) {
    glBegin(LCMGL_POINTS);
    for (int i=0; i<x.length; i++) { 
      glVertex3d(x[i], y[i], z[i]);
    }
    glEnd();
  }
  
  public synchronized void plot3(double[] x, double[] y, double[] z) {
    glBegin(LCMGL_LINES);
    for (int i=0; i<x.length-1; i++) { 
      glVertex3d(x[i], y[i], z[i]);
      glVertex3d(x[i+1], y[i+1], z[i+1]);
    }
    glEnd();    
  }

  public synchronized void rect(double[] xyz, double[] size, int filled) {
     try {
      _outs.writeByte(LCMGL_RECT); 
      _outs.writeDouble(xyz[0]);
      _outs.writeDouble(xyz[1]);
      _outs.writeDouble(xyz[2]);
      _outs.writeDouble(size[0]);
      _outs.writeDouble(size[1]);
      _outs.writeInt(filled);
    } catch(IOException xcp) {}
  }    

  public synchronized void drawArrow3d(double length, double head_width, double head_length, double body_width) {
    int slices = 20;
    int stacks = 20;

    double[] xyz = {0,0,0};

    //apply translations so the drawing is centered at origin along the x axis per bot_gl_draw_arrow_2d
    glPushMatrix();
    glTranslated(-length / 2, 0, 0);
    glRotated(90, 0, 1, 0);

    //draw body
    cylinder(xyz, body_width, body_width, length - head_length, slices, stacks);

    //draw head
    glTranslated(0, 0, length - head_length);
    cylinder(xyz, head_width, 0, head_length, slices, stacks);

    glPopMatrix();
  }

  public synchronized void drawVector3d(double[] origin, double[] vector) {
    double body_width = 0.0025;
    double head_width = 0.01;
    double head_length = 0.01;
    drawVector(origin,vector,body_width,head_width,head_length); 
  }

  public synchronized void scaleToViewerAr() { add0(LCMGL_SCALE_TO_VIEWER_AR); }

  public synchronized void drawVector(double[] origin, double[] vector,double body_width,double head_width,double head_length) {
    int slices = 20;
    int stacks = 20;

    double vector_norm = Math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2]);

    if (vector_norm>0) {
      glPushMatrix();
      
      glTranslated(origin[0],origin[1],origin[2]);

      // cross product with z axis to get rotation axis
      double[] rot_axis = {vector[1],-vector[0],0};
      if (rot_axis[0]==0&&rot_axis[1]==0) {
        // the vector is along the z axis, rot_angle will be 0 or 180 so use either x or y
        rot_axis[0] = 1;
      }
      double rot_angle = -Math.toDegrees(Math.acos(vector[2]/vector_norm));
      glRotated(rot_angle,rot_axis[0],rot_axis[1],rot_axis[2]);

      // make cylinder for body of vector
      double[] xyz = {0,0,0};
      cylinder(xyz,body_width,body_width,vector_norm-head_length,slices,stacks);
      
      // make cylinder for head of vector
      glTranslated(0,0,vector_norm-head_length);
      cylinder(xyz,head_width,0,head_length,slices,stacks);

      glPopMatrix();
    }

  }

  public synchronized void polygon(double[] x, double[] y, double[] z){
    glBegin(LCMGL_POLYGON);
    for(int i = 0;i<x.length;i++){
      glVertex3d(x[i],y[i],z[i]);
    }
    glEnd();
  }

  public synchronized void polyhedron(double[] x, double[] y, double[] z){
    for(int i =0;i<x.length;i++){
      for(int j=i;j<y.length;j++){
        for(int k = j;k<z.length;k++){
          glBegin(LCMGL_POLYGON);
          glVertex3d(x[i],y[i],z[i]);
          glVertex3d(x[j],y[j],z[j]);
          glVertex3d(x[k],y[k],z[k]);
          glEnd();
        }
      }
    }

  }
}
