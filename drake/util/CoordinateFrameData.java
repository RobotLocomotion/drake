package drake.util;

public class CoordinateFrameData {
  public double[] val;
  public double t;
  
  public CoordinateFrameData() {}
  
  public CoordinateFrameData(double t0, double[] x0) 
  {
    val = x0.clone();
    t = t0;
  }
}
