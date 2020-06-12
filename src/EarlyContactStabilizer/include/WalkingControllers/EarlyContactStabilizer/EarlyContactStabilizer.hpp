/**
 * @file EarlyContactStabilizer.hpp
 * @authors Milad Shafiee <milad.shafiee@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef WALKING_CONTROLLERS_EARLY_CONTACT_STABILIZER_H
#define WALKING_CONTROLLERS_EARLY_CONTACT_STABILIZER_H

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/sig/Vector.h>

//iDynTree
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/ConvexHullHelpers.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <WalkingControllers/iDynTreeUtilities/Helper.h>


namespace WalkingControllers
{

    class EarlyContactStabilizer
    {
        bool m_isEarlyContactStabilzerActive;
        double m_gravityAcceleration; /**< The gravity acceleration. */
        double m_mass; /**< Mass of the robot. */
        double m_modifiedFootVelocityZ; /**< Modified velocity of the foot in z direction. */
        iDynTree::Vector2 m_modifiedRollPitchFootAngle; /**< Modified angle of the roll and pitch of the foot. */
        iDynTree::Rotation m_modifiedTorsoOrientation;/**< Modified orientation  of the torso. */
        iDynTree::Vector3 m_footForceTorqueControlGains;/**< The vector of gains for position and orientation terms of the controller. */
        iDynTree::Vector3 m_footForceTorqueControlDerivativeGains;/**< The vector of gains for the derivative of the position and orientation terms of the controller. */
        iDynTree::Vector2 m_torsoOrientationControlDerivativeGains;/**< The vector of gains for the derivative terms in the torso controller. */
        iDynTree::Vector2 m_torsoOrientationControlGains;/**< The vector of gains for the torso controller. */
        iDynTree::Vector3 m_leftFootMappedForce;/**< The vector that includes the left foot  mapped force. */
        iDynTree::Vector3 m_leftFootMappedTorque;/**< The vector that includes the left foot mapped torque from desired zmp */
        iDynTree::Vector3 m_rightFootMappedForce;/**< The vector that includes the right foot  mapped force. */
        iDynTree::Vector3 m_rightFootMappedTorque;/**< The vector that includes the right foot mapped torque from desired zmp */

    public:

        /**
         * Config the early contact stabilizer.
         * @param config config of the simple DCM estimator;
         * @return true on success, false otherwise.
         */
        bool configure(const yarp::os::Searchable& config);

        /**
         * map the desired ZMP to the desired contact wrench.
         * @param zmp the vector of desired zmp position with respect to the inertial frame.
         * @param leftInContact The boolean that shows left foot is in contact or no.
         * @param rightInContact The boolean that shows rightft foot is in contact or no.
         * @param mass The total mass of the robot.
         * @param contactWrenchOrigin the position of the point that contact wrench has been calculated.
         * @return true/false in case of success/failure
         */
        bool mapDesiredZMPToDesiredContactWrench(const iDynTree::Position &desiredZMP, const bool &leftInContact, const bool &rightInContact, const double &mass, const iDynTree::Position &contactWrenchOrigin, const iDynTree::Position &leftFootPosition,const iDynTree::Position &rightFootPosition);

        /**
         * control the foot contact force in z direction.
         * @param desiredContactForceDerivativeZ the derivative of the desired contact force in z direction.
         * @param desiredContactForceZ the desired contact force in z direction.
         * @param measuredContactForceZ the measured contact force in z direction.
         * @return true/false in case of success/failure
         */
        bool footForceControl(const double& desiredContactForceDerivativeZ,const double& desiredContactForceZ, const double& measuredContactForceZ);

        /**
         * control the foot contact torque around x and y direction.
         * @param desiredContactTorqueDerivativeXY the derivative of the desired contact torque around x and y direction.
         * @param desiredContactTorqueXY the desired contact torque around x and y direction.
         * @param measuredContactTorqueXY the measured contact torque around x and y direction.
         * @return true/false in case of success/failure
         */
        bool footTorqueControl(const iDynTree::Vector2& desiredContactTorqueDerivativeXY,const iDynTree::Vector2& desiredContactTorqueXY, const iDynTree::Vector2& measuredContactTorqueXY);

        /**
         * control the upper-body posture.
         * @param desiredTorsoOrientation the desired  orientation of the torso.
         * @param measureTorsoOrientation the measured  orientation of the torso.
         * @return true/false in case of success/failure
         */
        bool uppderBodyPostureControl(const iDynTree::Rotation& desiredTorsoOrientation,const iDynTree::Rotation& measureTorsoOrientation);

        /**
         * Get the boolean to specify that the contact has been detected.
         * @return true/false in case of success/failure
         */
        bool isEarlyContactStabilizerActive();

        /**
         * Get the modified foot velocity in z direction.
         * @return Value of the modified foot velocity in z direction.
         */
        const double& getModifiedFootVelocity()const;

        /**
         * Get the modified orientation of the torso.
         * @return modified orientation of the torso.
         */
        const iDynTree::Rotation& getModifiedTorsoOrientation()const;

        /**
         * Get the modified roll and pitch angle of the foot in rad.
         * @return modified roll and pitch angle of the foot.
         */
        const iDynTree::Vector2& getModifiedFootRollPitch()const;
        const iDynTree::Vector3& getLeftFootMappedForce() const;
        const iDynTree::Vector3& getRightFootMappedForce() const;
        EarlyContactStabilizer();
    };
};

#endif
