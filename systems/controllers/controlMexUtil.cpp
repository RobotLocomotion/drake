#include "controlMexUtil.h"
#include "drakeMexUtil.h"

using namespace std;
using namespace Eigen;

std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> parseSupportData(const mxArray* supp_data) {
  double *logic_map_double;
  int nsupp = mxGetN(supp_data);
  if (mxGetM(supp_data) != 1) {
    mexErrMsgIdAndTxt("Drake:parseSupportData:BadInputs", "the support data should be a 1xN struct array");
  }
  int i, j;
  MatrixXd contact_pts;
  Vector4d contact_pt = Vector4d::Zero();
  contact_pt(3) = 1.0;
  int num_pts;
  std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> supports;
  const mxArray* pm;

  for (i = 0; i < nsupp; i++) {
    SupportStateElement se;

    se.body_idx = ((int) mxGetScalar(mxGetField(supp_data, i, "body_id"))) - 1;

    num_pts = mxGetN(mxGetField(supp_data, i, "contact_pts"));
    pm = mxGetField(supp_data, i, "support_logic_map");
    if (mxIsDouble(pm)) {
      logic_map_double = mxGetPrSafe(pm);
      assert(mxGetM(pm)==4);
      for (j = 0; j < 4; j++) {
        se.support_logic_map[j] = logic_map_double[j] != 0;
      }
    } else {
      mexErrMsgTxt("Please convert support_logic_map to double");
    }
    pm = mxGetField(supp_data, i, "contact_pts");
    contact_pts.resize(mxGetM(pm), mxGetN(pm));
    memcpy(contact_pts.data(), mxGetPrSafe(pm), sizeof(double)*mxGetNumberOfElements(pm));

    for (j = 0; j < num_pts; j++) {
      contact_pt.head(3) = contact_pts.col(j);
      se.contact_pts.push_back(contact_pt);
    }
    supports.push_back(se);
  }
  return supports;
}

