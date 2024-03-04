#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <towr/terrain/terrain_data.h>
#include <towr/terrain/examples/height_map_examples.h>

using namespace Eigen;
using namespace towr;

Eigen::VectorXd parseCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::vector<double> data;

    std::string line;
    std::getline(file, line); // Read the entire line
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        data.push_back(std::stod(cell));
    }

    Eigen::VectorXd vector(data.size());

    for (int i = 0; i < data.size(); ++i) {
        vector(i) = data[i];
    }

    return vector;
}

void ParseTerrainData(TerrainData& terrain_data){
  std::string folder_path = "/home/bolivar/Olympus-ws/towr_ros_catkin/src/tojr/towr/terrain_data";
  std::string x_file = folder_path + "/x.csv";
  std::string y_file = folder_path + "/y.csv";
  std::string z_file = folder_path + "/z.csv";
  
  const int nx = 3499;
  const int ny = 6981;
  double x_min = -3521.690825824481;
  double y_min = -2830992.8435943313;
  double x_max = 3467.741519556532;
  double y_max = -2817045.9488628395;
  double scale = 1.998122454494478;
  
  
  Eigen::VectorXd x = parseCSV(x_file);
  Eigen::VectorXd y = parseCSV(y_file);
  Eigen::VectorXd z = parseCSV(z_file);

  
  Eigen::VectorXd x_shift = x.array() - x_min - (x_max-x_min)/2;
  Eigen::VectorXd y_shift = y.array() - y_min - (y_max-y_min)/2;
  double x0 = -5.0;
  double y0 = -5.0;
  double length = 10.0;
  double width = 10.0;
  
  Eigen::VectorXd x_scaled = x_shift*0.1;
  Eigen::VectorXd y_scaled = y_shift*0.1;
  Eigen::VectorXd z_scaled = z* 0.1;
    
  scale*= 0.1;
  x_min = -(x_max-x_min)/2*0.01;
  y_min = -(y_max-y_min)/2*0.01;

  int i_min = floor(nx/2 - 30);
  int j_min = floor(ny/2 - 30);
  int i_max = floor(nx/2 + 30);
  int j_max = floor(ny/2 + 30);
  terrain_data.mu = 0.8;
  

  int k = 0;
  const int N = (i_max - i_min) * (j_max - j_min);
  Eigen::VectorXd x_data((i_max - i_min));
  Eigen::VectorXd y_data((j_max - j_min));
  Eigen::VectorXd z_data(N);
  
  for (int i = i_min; i < i_max; i++)   {
    for (int j = j_min; j < j_max; j++) {
      int global_index = j *nx + i;
      z_data(k) = z_scaled(global_index);
      k++;
    }
  }
  const int K = (i_max - i_min);
  for (int k = 0; k < K; k++) {
    x_data(k) =  -K/2*scale + k*scale;
  }

  const int L = (j_max - j_min);
    for (int l = 0; l < L; l++) {
        y_data(l) =  -L/2*scale + l*scale;
    }
  terrain_data.x = x_data;
  terrain_data.y = y_data;
  terrain_data.z = z_data.array() - z_data.minCoeff();
  
  //print bounds
  std::cout << "x_bounds: " << x_data.minCoeff() <<   " ," << x_data.maxCoeff() << std::endl;
  std::cout << "y_bounds: " << y_data.minCoeff() << ", " << y_data.maxCoeff() << std::endl;
  std::cout << "z_bounds: " << terrain_data.z.minCoeff() << " ," << terrain_data.z.maxCoeff() << std::endl;
}
    
int main() {
  TerrainData terrain_data;
  std::cout << 1<< std::endl;
  ParseTerrainData(terrain_data);
  std::cout << 2<< std::endl;

  TerrainFromData terrain(terrain_data);
  std::cout << 3<< std::endl;
  std::cout << "Terrain height at (0,0): " << terrain.GetHeight(0.0, 0.0) << std::endl;
  std::cout << "Terrain grad x at (0,0): " << terrain.GetHeightDerivWrtX(0.0, 0.0) << std::endl;
  std::cout << "Terrain grad y at (0,0): " << terrain.GetHeightDerivWrtY(0.0, 0.0) << std::endl;
  return 0;
}