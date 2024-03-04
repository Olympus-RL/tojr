#ifndef TERRAIN_DATA_H
#define TERRAIN_DATA_H


#include <Eigen/Core>

namespace towr {
 struct TerrainData {
            double mu; // friction coefficient
            Eigen::VectorXd x; // vector of nx evenly spaced x-coordinates
            Eigen::VectorXd y; // vector of ny evenly spaced y-coordinates
            Eigen::VectorXd z; // vector of nx*ny z-coordinates
        };
    }

#endif // TERRAIN_DATA_H
