/**
 * @file DCMSimpleEstimator.cpp
 * @authors Milad Shafiee <milad.shafiee@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// YARP
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

//iDynTree
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <WalkingControllers/EarlyContactStabilizer/EarlyContactStabilizer.hpp>
#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/iDynTreeUtilities/Helper.h>

using namespace WalkingControllers;

bool EarlyContactStabilizer::configure(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[EarlyContactStabilizer::configure] Empty configuration.";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "gains_Z_roll_pitch", m_footForceTorqueControlGains))
    {
        yError() << "[EarlyContactStabilizer::Configure] Unable to get the vector of the gains";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "gains_Z_roll_pitch_derivative", m_footForceTorqueControlDerivativeGains))
    {
        yError() << "[EarlyContactStabilizer::Configure] Unable to get the vector of the gains for derivative term";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "gains_torso_oreintation", m_torsoOrientationControlGains))
    {
        yError() << "[EarlyContactStabilizer::Configure] Unable to get the vector of the gains of the torso controller";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "gains_torso_oreintation_derivative", m_torsoOrientationControlDerivativeGains))
    {
        yError() << "[EarlyContactStabilizer::Configure] Unable to get the vector of the gains for derivative term of the torso controller";
        return false;
    }

    m_gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();
    m_isEarlyContactStabilzerActive=0;
    return true;
}

bool EarlyContactStabilizer::mapDesiredZMPToDesiredContactWrench(const iDynTree::Position &desiredZMP, const bool &leftInContact, const bool &rightInContact, const double &mass, const iDynTree::Position &contactWrenchOrigin, const iDynTree::Position &leftFootPosition,const iDynTree::Position &rightFootPosition)
{
    m_leftFootMappedForce.zero();
    m_rightFootMappedForce.zero();
    m_rightFootMappedTorque.zero();
    m_leftFootMappedTorque.zero();

    if(!leftInContact)
    {
        m_alpha=1;
        m_rightFootMappedForce(2)=-1*(m_alpha)*mass*9.81;
//        iDynTree::toEigen(m_rightFootMappedTorque)=iDynTree::skew(iDynTree::toEigen(contactWrenchOrigin)-iDynTree::toEigen(desiredZMP))*iDynTree::toEigen(m_rightFootMappedForce);
    }
    else if(!rightInContact)
    {
        m_alpha=0;
        m_leftFootMappedForce(2)=-1*(1-m_alpha)*mass*9.81;
//        iDynTree::toEigen(m_leftFootMappedTorque)=iDynTree::skew(iDynTree::toEigen(contactWrenchOrigin)-iDynTree::toEigen(desiredZMP))*iDynTree::toEigen(m_leftFootMappedForce);
    }
    else
    {
        m_alpha=abs(leftFootPosition(1)-desiredZMP(1))/abs(leftFootPosition(1)-rightFootPosition(1));

        m_rightFootMappedForce(2)=-1*(m_alpha)*mass*9.81;
        m_leftFootMappedForce(2)=-1*(1-m_alpha)*mass*9.81;
    }

    return true;
}

bool EarlyContactStabilizer::footForceControl(const double &desiredContactForceDerivativeZ, const double &desiredContactForceZ, const double &measuredContactForceZ)
{
    m_modifiedFootVelocityZ=-1*((m_footForceTorqueControlGains(0)*desiredContactForceZ-measuredContactForceZ)+m_footForceTorqueControlDerivativeGains(0)*desiredContactForceDerivativeZ);
    return true;
}

bool EarlyContactStabilizer::footTorqueControl(const iDynTree::Vector2 &desiredContactTorqueDerivativeXY, const iDynTree::Vector2 &desiredContactTorqueXY, const iDynTree::Vector2 &measuredContactTorqueXY)
{
    return true;
}

bool EarlyContactStabilizer::uppderBodyPostureControl(const iDynTree::Rotation &desiredTorsoOrientation, const iDynTree::Rotation &measureTorsoOrientation)
{
    return true;
}

const double& EarlyContactStabilizer::getModifiedFootVelocity() const
{
    return m_modifiedFootVelocityZ;
}

const iDynTree::Vector2& EarlyContactStabilizer::getModifiedFootRollPitch() const
{
    return m_modifiedRollPitchFootAngle;
}

const iDynTree::Rotation& EarlyContactStabilizer::getModifiedTorsoOrientation() const
{
    return m_modifiedTorsoOrientation;
}

const iDynTree::Vector3& EarlyContactStabilizer::getLeftFootMappedForce() const
{
    return m_leftFootMappedForce;
}

const iDynTree::Vector3& EarlyContactStabilizer::getRightFootMappedForce() const
{
    return m_rightFootMappedForce;
}

bool EarlyContactStabilizer::isEarlyContactStabilizerActive()
{
    return m_isEarlyContactStabilzerActive;
}

bool EarlyContactStabilizer::getContactState(footContactState &rightFootContactState,footContactState& leftFootContactState,const iDynTree::Wrench& rightWrench,
                                             const iDynTree::Wrench& leftWrench,const bool &leftInContact,const bool &rightInContact)
{
        if(rightInContact  && !(rightWrench.getLinearVec3()(2)>20))
        {
            rightFootContactState=footContactState::late;
        }
        else if(!rightInContact && (rightWrench.getLinearVec3()(2)>20))
        {
            rightFootContactState=footContactState::early;
        }
        else
        {
            rightFootContactState=footContactState::onTime;
        }

        if(leftInContact  && !(leftWrench.getLinearVec3()(2)>20))
        {
            leftFootContactState=footContactState::late;
        }
        else if(!rightInContact && (leftWrench.getLinearVec3()(2)>20))
        {
            leftFootContactState=footContactState::early;
        }
        else
        {
            leftFootContactState=footContactState::onTime;
        }

    return true;
}

const double& EarlyContactStabilizer::getAlpha() const
{
    return m_alpha;
}
