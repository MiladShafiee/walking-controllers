/**
 * @file WalkingDCMModelPredictiveController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef STEP_ADAPTATOR_HPP
#define STEP_ADAPTATOR_HPP

// std
#include <memory>

// eigen
#include <Eigen/Sparse>

// iDynTree
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/ConvexHullHelpers.h>

// yarp
#include <yarp/os/Value.h>

#include <unordered_map>
#include <deque>

// solver
#include <QPSolver.hpp>

/**
 * StepAdaptator class contains the controller instances.
 */
class StepAdaptator
{
    /**
     * The value of change of the ZMP during single support phase
     */
    double m_delta;
    /**
     * The vector of the tolerance of the bound of inequality constraint relative to nominal value.
     */
    iDynTree::Vector4 m_constraintTolerance;

    /**
     * The vector of gains of the QP problem.
     */
    iDynTree::Vector4 m_gainVector;
    /**
     * The hessian matrix of the QP problem. It is the same for all controllers.
     */
    iDynSparseMatrix m_hessianMatrix;

    int m_inputSize;  /**< Size of the input vector. It is equal to 3 now!!!!. */
    int m_numberOfConstraint;  /**< Size of the input vector. It is equal to 5 now!!!!. */

    std::pair<bool, bool> m_feetStatus; /**< Current status of the feet. Left and Right. True is used
                                           if the foot is in contact. */

    bool m_isSolutionEvaluated{false}; /**< True if the solution is evaluated. */

    /**
     * Pointer to the current QPSolver.
     * A new MPC solver is initialized when a new phase occurs.
     */
    std::shared_ptr<QPSolver> m_currentQPSolver;

    /**
     * Vector containing the output of the setp adaptator.
 */
    iDynTree::Vector6 m_outputStepAdaptator;

public:

    /**
     * Initialize the method
     * @param config yarp searchable configuration variable.
     * @return true/false in case of success/failure
     */
    bool initialize(const yarp::os::Searchable& config);


    /**
     * Solve the Optimization problem. If the QPSolver is not set It will be initialized.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the output of the controller.
     * @param controllerOutput is the vector containing the output the controller.
     * @return true/false in case of success/failure.
     */
    bool getControllerOutput(iDynTree::Vector6& controllerOutput);

    /**
     * Reset the controller
     */
    void reset();
    /**
     * Run the step adaptation and set constraints and gradient vector and solve the QP problem
     * @param nominalValuesVector Vector that includes the Desired Value of DCM at the landing moment of foot, StepTiming and next StepPosition and next DCM Offset;
     * @param currentValuesVector This vector includes the current value of real ZMP, real DCM and delta(the distance that ZMP moves in the SS phase)   ;
     * @return true/false in case of success/failure.
     */
    bool RunStepAdaptator(const iDynTree::Vector4 &nominalValues, const iDynTree::Vector3 &currentValues);
};

#endif
