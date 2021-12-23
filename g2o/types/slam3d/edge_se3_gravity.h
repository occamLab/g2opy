/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * Adapted code from HDL graph slam:
 * https://github.com/koide3/hdl_graph_slam/blob/master/include/g2o/edge_se3_priorvec.hpp
 */

#ifndef G2O_EDGE_SE3_GRAVITY_H_
#define G2O_EDGE_SE3_GRAVITY_H_

#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"


  /*! \class EdgeSE3Gravity
   * \brief g2o edge with gravity constraint
   */
  class EdgeSE3Gravity : public g2o::BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, g2o::VertexSE3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3Gravity(){
    	information().setIdentity();
    }
    virtual bool read(std::istream& is) {return false;} // not implemented
    virtual bool write(std::ostream& os) const {return false;} // not implemented

    // return the error estimate as a 3-vector
    void computeError(){
    	const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

		Eigen::Vector3d direction = _measurement.head<3>();
		Eigen::Vector3d measurement = _measurement.tail<3>();

		Eigen::Vector3d ea;
		// Note: while the name is not very intuitive, .linear() extracts the rotation part of the transform (assuming the transform contains only rotation and translation)
		// t contains the estimate of the up vector measured in the phone frame
		Eigen::Matrix3d t = v1->estimate().linear().transpose();
		Eigen::Vector3d estimate = t * direction;
		_error = estimate - measurement;

		printf("%d : measured=%f %f %f est=%f %f %f error=%f %f %f\n", v1->id(),
		  measurement[0], measurement[1], measurement[2],
		  estimate[0], estimate[1], estimate[2],
		  _error[0], _error[1], _error[2]);
    }

    // 6 values:
    // [0:2] Up vector in robot frame
    // [3:5] Observed gravity vector in world frame
    virtual void setMeasurement(const Eigen::Matrix<double, 6, 1>& m){
    	_measurement.head<3>() = m.head<3>().normalized();
        _measurement.tail<3>() = m.tail<3>().normalized();
    }
  };
#endif
