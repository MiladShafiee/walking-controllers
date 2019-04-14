

#ifndef QP_SOLVER_HPP
#define QP_SOLVER_HPP

// std
#include <deque>

// iDynTree
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/VectorDynSize.h>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

#include <Utils.hpp>

/**
 * QPSolver class
 */
class QPSolver
{
    /**
     * Pointer to the optimization solver
     */
    std::unique_ptr<OsqpEigen::Solver> m_QPSolver;

    Eigen::VectorXd m_lowerBound; /**< Lower bound vector. */
    Eigen::VectorXd m_upperBound; /**< Upper bound vector. */

    int m_inputSize; /**< Size of the controlled input vector (2). */

    /**
     * Evaluation and returning the the part of constraints matrix(A) that is related to the inequality constraints
     * @return the part of constraints matrix(A) that is related to the inequality constraints   .
     */
    Eigen::Matrix<double,4,3> evaluateInEqualityPartOfConstraintsMatrix();
    /**
     * Evaluation and returning constraints matrix(A) related to equality and inequality constraints(C<Ax<B)
     * @param currentValuesVector This vector includes the current value of real ZMP, real DCM and delta(the distance that ZMP moves in the SS phase)   ;
     * @return the constraints matrix(A).
     */
       Eigen::SparseMatrix<double> evaluateConstraintsMatrix(const iDynTree::Vector3 &currentValuesVector);
public:

    /**
     * Constructor.
     * @param numberOfAllConstraints number of equality and inequality constraints!
     * @param inputSize size of the controlled input vector;
     */
    QPSolver(const int& inputSize,
             const int& numberOfAllConstraints);

    /**
     * Set the hessian matrix.
     * Please do not call this function to update the hessian matrix! It can be set only once.
     * @param hessian hessian matrix.
     * @return true/false in case of success/failure.
     */
    bool setHessianMatrix(const iDynTree::Vector4& alphaVector);

    /**
     * Set or update the linear constraints matrix(A) related to equality and inequality constraints(C<Ax<B)
     * If the solver is already set the linear constraints matrix is updated otherwise it is set for
     * the first time.
     * @param currentValuesVector This vector includes the current value of real ZMP, real DCM and delta(the distance that ZMP moves in the SS phase)   ;
     * @return true/false in case of success/failure.
     */
    bool setConstraintsMatrix(const iDynTree::Vector3& currentValuesVector);

    /**
     * Set or update the gradient
     * @param gainsVector  vector that includes cost function gains
     * @param nominalValuesVector Vector that includes the Desired Value of DCM at the landing moment of foot, StepTiming and next StepPosition and next DCM Offset;
     * @return true/false in case of success/failure.
     */
    bool setGradientVector(const iDynTree::Vector4 &alphaVector,const iDynTree::Vector4& nominalValuesVector );

    /**
     * Get the primal variable.
     * @param primalVariable primal variable vector
     * @return true/false in case of success/failure.
     */
    bool getPrimalVariable(Eigen::VectorXd& primalVariable);

    /**
     * Set the primal variable.
     * @param primalVariable primal variable vector
     * @return true/false in case of success/failure.
     */
    bool setPrimalVariable(const Eigen::VectorXd& primalVariable);

    /**
     * Get the state of the solver.
     * @return true if the solver is initialized false otherwise.
     */
    bool isInitialized();

    /**
     * Initialize the solver.
     * @return true/false in case of success/failure.
     */
    bool initialize();

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the solver solution
     * @return the entire solution of the solver
     */
    iDynTree::VectorDynSize getSolution();

    /**
     * Set or update the lower and the upper bounds
     * @param nominalValuesVector Vector that includes the Desired Value of DCM at the landing moment of foot, StepTiming and next StepPosition and next DCM Offset;
     * @param currentValuesVector This vector includes the current value of real ZMP, real DCM and delta(the distance that ZMP moves in the SS phase)   ;
     * @param tolerenceOfBounds This vector includes the tolerence between nominal value and the maximum and minimum value constraint    ;
     * @return true/false in case of success/failure.
     */
    bool setBoundsVectorOfConstraints(const iDynTree::Vector4& nominalValuesVector, const iDynTree::Vector3 &currentValuesVector, const iDynTree::Vector4& tolerenceOfBounds);
};

#endif
