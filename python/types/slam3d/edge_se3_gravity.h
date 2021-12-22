#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <g2o/types/slam3d/edge_se3_gravity.h>

namespace py = pybind11;
using namespace pybind11::literals;


namespace g2o {

void declareEdgeSE3Gravity(py::module & m) {
    templatedBaseEdge<3, Eigen::Matrix<double, 6, 1> >(m, "_3_Eigen6dVec_BaseEdge");
    templatedBaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, VertexSE3>(m, "_3_Eigen6dVec_VertexSE3");
    py::class_<EdgeSE3Gravity, BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, VertexSE3>>(m, "EdgeSE3Gravity")
        .def(py::init<>())
    
        .def("compute_error", &EdgeSE3Gravity::computeError)
   	.def("set_measurement", &EdgeSE3Gravity::setMeasurement)
    //    .def("set_measurement_data", &EdgeSE3::setMeasurementData)
     //   .def("get_measurement_data", &EdgeSE3::getMeasurementData)
      //  .def("measurement_dimension", &EdgeSE3::measurementDimension)
      //  .def("linearize_oplus", &EdgeSE3::linearizeOplus)
    ;
}

}
