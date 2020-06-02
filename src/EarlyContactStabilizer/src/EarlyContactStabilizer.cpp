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

    return true;
}

bool EarlyContactStabilizer::mapDesiredZMPToDesiredContactWrench(const iDynTree::Vector2 &desiredZMP, const bool &leftInContact, const bool &rightInContact, const double &mass, const iDynTree::Position &contactWrenchOrigin)
{
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

bool EarlyContactStabilizer::isEarlyContactStabilizerActive()
{
    return m_isEarlyContactStabilzerActive;
}
