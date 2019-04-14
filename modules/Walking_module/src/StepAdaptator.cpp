

// std 
#define NOMINMAX
#include <algorithm>

// yarp
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/Core/Direction.h>

#include <StepAdaptator.hpp>
#include <Utils.hpp>

bool StepAdaptator::initialize(const yarp::os::Searchable &config){
//This method will be called in configure function of walkingmodule
    yarp::os::Value gains = config.find("cost_function_gains");

    if(!(gains.isNull()))
    {
        if (!gains.isList() || !gains.asList())
        {
            yError()<<"[StepAdaptator::initialize] Unable to read the cost function gains list.";
            return false;
        }
        yarp::os::Bottle *gainsVector = gains.asList();

        for(int i = 0; i < gainsVector->size(); ++i)
        {
            if(!gainsVector->get(i).isDouble())
            {
                yError("[StepAdaptator::initialize] The gains value is expected to be a double");
            }
            m_gainVector(i)=gainsVector->get(i).asDouble();
        }
    }




    yarp::os::Value constraintTolerance = config.find("tolerence_of_bounds");
    if(!(gains.isNull()))
    {
        if (!gains.isList() || !gains.asList())
        {
            yError()<<"[StepAdaptator::initialize] Unable to read the m_constraintTolerance list.";
            return false;
        }
        yarp::os::Bottle *constraintToleranceVector = gains.asList();

        for(int i = 0; i < constraintToleranceVector->size(); ++i)
        {
            if(!constraintToleranceVector->get(i).isDouble())
            {
                yError("[StepAdaptator::initialize] The tolerence of bounds of constraints value is expected to be a double");
            }
            m_constraintTolerance(i)=constraintToleranceVector->get(i).asDouble();
        }
    }


    // = config.check("delta");

    if(!YarpHelper::getNumberFromSearchable(config, "delta", m_delta))
    {
        yError() << "[StepAdaptator::initialize] Unable get delta(double) from searchable.";
        return false;
    }

    // reset the solver
    reset();
    m_inputSize=3;
    m_numberOfConstraint=5;

    m_currentQPSolver = std::make_shared<QPSolver>(m_inputSize,m_numberOfConstraint);
    m_currentQPSolver->setHessianMatrix(m_gainVector);// we only set Hessian one time because it deponds only on the const function gains that are constant!

    return true;

}


bool StepAdaptator::RunStepAdaptator(const iDynTree::Vector4& nominalValues,const iDynTree::Vector3& currentValues)
{
    if (!m_currentQPSolver->setGradientVector(m_gainVector,nominalValues)) {
        yError() << "[StepAdaptator::RunStepAdaptator] Unable to set the Gradient Vector";
        return false;
    }
    if (!m_currentQPSolver->setBoundsVectorOfConstraints(nominalValues,currentValues,m_constraintTolerance)) {
        yError() << "[StepAdaptator::RunStepAdaptator] Unable to set the Bounds Vector Of Constraints";
        return false;
    }

    if (!m_currentQPSolver->setConstraintsMatrix(currentValues)) {
        yError() << "[StepAdaptator::RunStepAdaptator] Unable to set the constraint matrix";
        return false;
    }

    return true;
}



bool StepAdaptator::solve()
{
    m_isSolutionEvaluated = false;
    if(!m_currentQPSolver->isInitialized())
    {
        if(!m_currentQPSolver->initialize())
        {
            yError() << "[StepAdaptator::solve] Unable to initialize the QP solver.";
            return false;
        }
    }

    if(!m_currentQPSolver->solve())
    {
        yError() << "[StepAdaptator::solve] Unable to solve the step adaptation problem.";
        return false;
    }

    iDynTree::VectorDynSize solution = m_currentQPSolver->getSolution();
    m_outputStepAdaptator(0) = solution(0);
    m_outputStepAdaptator(1) = solution(1);
    m_outputStepAdaptator(2) = solution(2);

    m_isSolutionEvaluated = true;
    return true;
}

bool StepAdaptator::getControllerOutput(iDynTree::Vector6& controllerOutput)
{
    if(!m_isSolutionEvaluated)
    {
        yError() << "[StepAdaptator::getControllerOutput] The solution is not evaluated. "
                 << "Please call 'solve()' method.";
        return false;
    }

    m_isSolutionEvaluated = false;
    controllerOutput = m_outputStepAdaptator;
    return true;
}


void StepAdaptator::reset(){
    m_isSolutionEvaluated = false;
}
