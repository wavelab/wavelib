#include "wave/optimization/ceres/point_to_point_residual.hpp"

namespace wave {

bool AnalyticalPointToPoint::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    // modelled as P1 = T*P2
    // r = T*P2 - P1;
    // parameters in order:
    // R11 R21 R31 R12 R22 R32 R13 R23 R33 X Y Z
    r[0] = parameters[0] * this->P2[0] + parameters[3] * this->P2[1] + parameters[6] * this->P2[2] + parameters[9] - this->P1[0];
    r[1] = parameters[1] * this->P2[0] + parameters[4] * this->P2[1] + parameters[7] * this->P2[2] + parameters[10] - this->P1[1];
    r[2] = parameters[2] * this->P2[0] + parameters[5] * this->P2[1] + parameters[8] * this->P2[2] + parameters[11] - this->P1[2];

    if((jacobians != NULL) && (jacobians[0] != NULL)) {
        jacobians[0][0] = this->P2[0];
        jacobians[0][1] = 0;
        jacobians[0][2] = 0;
        jacobians[0][3] = this->P2[1];
        jacobians[0][4] = 0;
        jacobians[0][5] = 0;
        jacobians[0][6] = this->P2[2];
        jacobians[0][7] = 0;
        jacobians[0][8] = 0;
        jacobians[0][9] = 1;
        jacobians[0][10] = 0;
        jacobians[0][11] = 0;

        jacobians[0][12] = 0;
        jacobians[0][13] = this->P2[0];
        jacobians[0][14] = 0;
        jacobians[0][15] = 0;
        jacobians[0][16] = this->P2[1];
        jacobians[0][17] = 0;
        jacobians[0][18] = 0;
        jacobians[0][19] = this->P2[2];
        jacobians[0][20] = 0;
        jacobians[0][21] = 0;
        jacobians[0][22] = 1;
        jacobians[0][23] = 0;

        jacobians[0][24] = 0;
        jacobians[0][25] = 0;
        jacobians[0][26] = this->P2[0];
        jacobians[0][27] = 0;
        jacobians[0][28] = 0;
        jacobians[0][29] = this->P2[1];
        jacobians[0][30] = 0;
        jacobians[0][31] = 0;
        jacobians[0][32] = this->P2[2];
        jacobians[0][33] = 0;
        jacobians[0][34] = 0;
        jacobians[0][35] = 1;
    }
}

}