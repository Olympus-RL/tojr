#ifndef OPTJUMP_PYTHON_H
#define OPTJUMP_PYTHON_H



#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <towr/optjump.h>



namespace py = pybind11;

namespace towr {



PYBIND11_MODULE(opt_jump, m) {
    py::class_<OptJump, std::shared_ptr<OptJump>>(m, "OptJump")
        .def(py::init())
        .def("set_initial_base_state", &OptJump::SetInitialBaseState)
        .def("set_initial_EE_state", &OptJump::SetInitialEEState)
        .def("set_jump_length", &OptJump::SetJumpLength)
        .def("solve", &OptJump::Solve)
        .def("set_takeoff_duration", &OptJump::SetTakeoffDuration)
        .def("get_base_state", &OptJump::GetBaseState)
        .def("get_EE_pos", &OptJump::GetEEPos);
}

} // namespace towr

#endif // OPTJUMP_PYTHON_H∂