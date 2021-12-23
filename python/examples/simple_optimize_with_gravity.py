# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/simple_optimize/simple_optimize.cpp

import numpy as np
import g2o
from g2o import VertexSE3, Isometry3d, Quaternion, EdgeSE3, EdgeSE3Gravity

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-n', '--max_iterations', type=int, default=10, help='perform n iterations')
parser.add_argument('-i', '--input', type=str, default='sphere2500.g2o', help='input file')
parser.add_argument('-o', '--output', type=str, default='', help='save resulting graph as file')
args = parser.parse_args()


def populate_graph(optimizer):
    vert1_estimate = Isometry3d()
    rot_angle = np.pi/8
    q = Quaternion(np.cos(rot_angle/2), 0.0*np.sin(rot_angle/2), 1.0*np.sin(rot_angle/2), 0.0*np.sin(rot_angle/2))
    vert1_estimate.set_rotation(q)
    vert = VertexSE3()
    vert.set_id(0)
    vert.set_estimate(vert1_estimate)

    vert2_estimate = Isometry3d()
    vert2_estimate.set_rotation(q)
    # this is a translation in the rotated coordinate system
    vert2_estimate.set_translation(q * np.array([0.0, 0.0, 1.0]))
    vert2 = VertexSE3()
    vert2.set_id(1)
    vert2.set_estimate(vert2_estimate)

    optimizer.add_vertex(vert)
    optimizer.add_vertex(vert2)

    edge_measurement = Isometry3d()
    edge_measurement.set_translation(np.array([0.0, 0.0, 1.0]))
    edge = EdgeSE3()
    edge.set_measurement(edge_measurement)
    edge.set_vertex(0, vert)
    edge.set_vertex(1, vert2)
    edge.set_id(0)

    gravity_edge = EdgeSE3Gravity()
    gravity_edge.set_id(1)
    # The gravity edge
    gravity_edge.set_measurement(np.array([0.0, 1.0, 0.0, 0.1, np.sqrt(1.0-0.1**2), 0.0]))
    # this does not seem to be optimizing very well
    gravity_edge.set_information(np.eye(3))
    gravity_edge.set_vertex(0, vert)
    print(type(gravity_edge.measurement()))
    #optimizer.add_edge(gravity_edge)
    print("edge error", edge.compute_error())
    optimizer.add_edge(edge)


def main():
    solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)

    optimizer = g2o.SparseOptimizer()
    optimizer.set_verbose(True)
    optimizer.set_algorithm(solver)
    populate_graph(optimizer)
    print('num vertices:', len(optimizer.vertices()))
    print('num edges:', len(optimizer.edges()), end='\n\n')
    print(optimizer.vertex(0).estimate().translation())
    print(optimizer.vertex(0).estimate().rotation().matrix())
    print(optimizer.vertex(1).estimate().translation())
    print(optimizer.vertex(1).estimate().rotation().matrix())
    optimizer.initialize_optimization()
    optimizer.optimize(args.max_iterations)
    print("result 1")
    print(optimizer.vertex(0).estimate().translation())
    print(optimizer.vertex(0).estimate().rotation().matrix())
    print("result 2")
    print(optimizer.vertex(1).estimate().translation())
    print(optimizer.vertex(1).estimate().rotation().matrix())


    if len(args.output) > 0:
        optimizer.save(args.output)


if __name__ == '__main__':
    main()
