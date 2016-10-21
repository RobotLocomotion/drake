// faster alternative to Matlab ppval
// @author Michael Kaess

package drake.matlab.systems.trajectories.dev;

public class JavaPP {

  double[] m_breaks;
  double[][] m_coefs;
  int m_order;
  int m_dim1;
  int m_dim2;
  int m_cached_idx;

  public JavaPP(double[] breaks, double[][] coefs, int order, int[] dims) {
    m_breaks = breaks.clone();
    m_coefs = new double[coefs.length][];
    for (int i=0; i<coefs.length; i++) {
      m_coefs[i] = coefs[i].clone();
    }
    m_order = order;
    // dims can have one or two entries
    m_dim1 = dims[0];
    if (dims.length > 1)
      m_dim2 = dims[1];
    else
      m_dim2 = 1;
    m_cached_idx = -1;
  }

  // evaluate the PP at locate t
  // to be optimized for multiple access to same PP, with temporal coherence
  public double[][] eval(double t) {

    double[][] r = new double[m_dim1][m_dim2];

    // use cached index if possible
    int idx;
    if (m_cached_idx >= 0 && m_cached_idx < m_breaks.length // valid m_cached_idx?
        && t < m_breaks[m_cached_idx+1] // still in same interval?
        && ((m_cached_idx == 0 ) || (t >= m_breaks[m_cached_idx]))) { // left up to -infinity
      idx = m_cached_idx;
    } else {
      // search for the correct interval
      idx = java.util.Arrays.binarySearch(m_breaks, t);
      if (idx==m_breaks.length) idx=m_breaks.length-1; // the last break does not matter
      if (idx<0) idx = -idx-1;
      idx = idx -1;
      if (idx<0) idx=0;
      // handle degenerate case of duplicate breaks consistently with Matlab
      while (m_breaks[idx]==m_breaks[idx+1] && idx<m_breaks.length-2) idx++;
      m_cached_idx = idx;
    }

    int base = idx*m_dim1*m_dim2;
    for (int l=0; l<m_dim2; l++) {
      for (int k=0; k<m_dim1; k++) {
        r[k][l] = m_coefs[base+k+l*m_dim1][0];
      }
    }
    double local = t - m_breaks[idx];
    for (int i=2; i<=m_order; i++) {
      for (int l=0; l<m_dim2; l++) {
        for (int k=0; k<m_dim1; k++) {
          r[k][l] = local*r[k][l] + m_coefs[base+k+l*m_dim1][i-1];
        }
      }
    }
        
    return r;
  }

  public void shiftTime(double offset) {
    for (int i=0; i<m_breaks.length; i++) {
      m_breaks[i] = m_breaks[i] + offset;
    }
  }

  public void uminus() {
    for(int i=0; i<m_order; i++) {
      for (int j=0; j<m_dim1*m_dim2*(m_breaks.length-1); j++) {
        m_coefs[j][i] = -m_coefs[j][i];
      }
    }
  }
}
