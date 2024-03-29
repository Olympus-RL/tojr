#include <eigen3/Eigen/Dense>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

using Vector3d = Eigen::Vector3d;

Vector3d add(Vector3d i, Vector3d j) {
    return i + j;
}

PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function that adds two numbers");
}