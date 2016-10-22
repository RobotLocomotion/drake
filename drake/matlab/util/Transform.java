package drake.matlab.util;

// java versions of the most useful transformations in the matlab directory, 
// unit tested to be equivalent to their matlab counter-parts

import static java.lang.Math.*;

public class Transform
{
  public static double[] rpy2quat(double[] rpy) 
  {
    if (rpy.length != 3) { 
      throw new IllegalArgumentException("rpy must have 3 elements");
    }
    
    double[] q = new double[4];
    
    rpy[0] /= 2.0;  rpy[1] /= 2.0;  rpy[2] /= 2.0;

        
    q[0] = cos(rpy[0])*cos(rpy[1])*cos(rpy[2]) + sin(rpy[0])*sin(rpy[1])*sin(rpy[2]); 
    q[1] = sin(rpy[0])*cos(rpy[1])*cos(rpy[2]) - cos(rpy[0])*sin(rpy[1])*sin(rpy[2]);
    q[2] = cos(rpy[0])*sin(rpy[1])*cos(rpy[2]) + sin(rpy[0])*cos(rpy[1])*sin(rpy[2]);
    q[3] = cos(rpy[0])*cos(rpy[1])*sin(rpy[2]) - sin(rpy[0])*sin(rpy[1])*cos(rpy[2]);

    double norm=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (abs(norm)>1e-12) {
      q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
    return q;
  }

  public static double[] quat2rpy(double[] q)
  {
    if (q.length != 4) { 
      throw new IllegalArgumentException("q must have 4 elements");
    }
    
    double norm=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (abs(norm)>1e-12) {
      q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }

    double w=q[0], x=q[1], y=q[2], z=q[3];
    
    double[] rpy = new double[3];
    rpy[0] = atan2(2.0*(w*x + y*z), w*w + z*z -(x*x +y*y));
    rpy[1] = asin(2*(w*y - z*x));
    rpy[2] = atan2(2*(w*z + x*y), w*w + x*x-(y*y+z*z));
    
    return rpy;
  }
  
  public static double[] rpydot2angularvel(double[] rpy, double[] rpydot) 
  {
    if (rpy.length != 3) { 
      throw new IllegalArgumentException("rpy must have 3 elements");
    }
    if (rpydot.length != 3) { 
      throw new IllegalArgumentException("rpydot must have 3 elements");
    }
    
    double[] omega = new double[3];
    
    omega[0] = cos(rpy[1])*cos(rpy[2])*rpydot[0] - sin(rpy[2])*rpydot[1];
    omega[1] = cos(rpy[1])*sin(rpy[2])*rpydot[0] + cos(rpy[2])*rpydot[1];
    omega[2] = -sin(rpy[1])*rpydot[0] + rpydot[2];
    
    return omega;
  }
  
  public static double[] angularvel2rpydot(double[] rpy, double[] omega) 
  {
    if (rpy.length != 3) { 
      throw new IllegalArgumentException("rpy must have 3 elements");
    }
    if (omega.length != 3) { 
      throw new IllegalArgumentException("rpydot must have 3 elements");
    }
   
    double[] rpydot = new double[3];
    
    rpydot[0] = cos(rpy[2])/cos(rpy[1])*omega[0] + sin(rpy[2])/cos(rpy[1])*omega[1];
    rpydot[1] = -sin(rpy[2])*omega[0] + cos(rpy[2])*omega[1];
    rpydot[2] = cos(rpy[2])*tan(rpy[1])*omega[0] + tan(rpy[1])*sin(rpy[2])*omega[1] + omega[2];
    
    return rpydot;
  }  
}
