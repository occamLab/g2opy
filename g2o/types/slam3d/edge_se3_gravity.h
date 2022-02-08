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
 * Adapted from: https://github.com/introlab/rtabmap/blob/c4d127cae40e8f92db07c05c0dadcaef4cb97d14/corelib/src/
 * optimizer/g2o/edge_se3_gravity.h (which adapted code from HDL graph slam:
 * https://github.com/koide3/hdl_graph_slam/blob/master/include/g2o/edge_se3_priorvec.hpp)
 */

#ifndef G2O_EDGE_SE3_GRAVITY_H_
#define G2O_EDGE_SE3_GRAVITY_H_

#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/se3_ops.h"
#include "g2o/types/sba/types_sba.h"
#include <Eigen/Geometry>


  /*! \class EdgeSE3Gravity
   * \brief g2o edge with gravity constraint.
   */
  class EdgeSE3Gravity : public g2o::BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, g2o::VertexSE3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3Gravity(){
    	information().setIdentity();
    }
    virtual bool read(std::istream& is) {return false;} // not implemented
    virtual bool write(std::ostream& os) const {return false;} // not implemented

      /**
       * \brief Set a flag that tells the `computeError` function to treat the 0th vertex as a VertexSE3ExpMap and
       * therefore not invert the rotation matrix before multiplying by the direction vector.
       */
      void setOdometryIsVertexSE3Expmap() { _odometry_is_se3_expmap = true; }

      // return the error estimate as a 3-vector
      void computeError(){
          Eigen::Vector3d direction = _measurement.head<3>();
          Eigen::Vector3d measurement = _measurement.tail<3>();

          auto v1_before_cast = _vertices[0];

          if (_odometry_is_se3_expmap) {
              const g2o::VertexSE3Expmap* v1_se3_expmap = static_cast<const g2o::VertexSE3Expmap*>(v1_before_cast);
              Eigen::Vector3d estimate = v1_se3_expmap->estimate().rotation().matrix() * direction;
              _error = estimate - measurement;
          } else {  // Assume it is a g2o::VertexSE3 instance
              const g2o::VertexSE3* v1_se3 = static_cast<const g2o::VertexSE3*>(v1_before_cast);

              // Note: while the name is not very intuitive, .linear() extracts the rotation part of the transform
              // (assuming the transform contains only rotation and translation)
              // estimate contains the up vector measured in the phone frame
              Eigen::Vector3d estimate = v1_se3->estimate().linear().transpose() * direction;
              _error = estimate - measurement;
          }
    }

    // 6 values:
    // [0:2] Up vector in robot frame
    // [3:5] Observed gravity vector in world frame
    virtual void setMeasurement(const Eigen::Matrix<double, 6, 1>& m){
    	_measurement.head<3>() = m.head<3>().normalized();
        _measurement.tail<3>() = m.tail<3>().normalized();
    }

  protected:
      bool _odometry_is_se3_expmap = false;
  };
#endif
