/**
 * @file LinearInvertedPendulumModel.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLERS_DCM_SIMPLE_ESTIMATOR_H
#define WALKING_CONTROLLERS_DCM_SIMPLE_ESTIMATOR_H

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/sig/Vector.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

//iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Twist.h>


namespace WalkingControllers
{
    class DCMSimpleEstimator
    {
        double m_omega; /**< Inverted time constant of the 3D-LIPM. */
        double m_mass;
        std::unique_ptr<iCub::ctrl::Integrator> m_dcmVelocityIntegrator{nullptr}; /**< CoM integrator object. */
        iDynTree::Vector2 m_dcmEstimatedPosition; /**< Position of the DCM. */
        iDynTree::Vector2 m_dcmPosition; /**< Position of the DCM. */
        iDynTree::Vector2 m_comPosition; /**< Position of the CoM. */
        iDynTree::Vector2 m_dcmVelocity; /**< Velocity of the dcm. */

    public:

        /**
         * Initialize the DCMEstimator.
         * @param config config of the 3D-LIPM;
         * @return true on success, false otherwise.
         */
        bool initialize(const yarp::os::Searchable& config, const iDynTree::Model modelLoader);

        /**
         * Get the position of the DCM.
         * @return position of the DCM.
         */
        const iDynTree::Vector2& getDCMPosition() const;

        /**
         * Get the velocity of the CoM.
         * @return velocity of the CoM.
         */
        const iDynTree::Vector2& getCoMVelocity() const;

        /**
         * Reset the Estimator
         * @param initialValue initial position of the DCM
         * @return true/false in case of success/failure
         */
        bool pendulumEstimator(iDynTree::Rotation footOrientation, iDynTree::Vector3 zmp, iDynTree::Vector3 com, iDynTree::LinVelocity CoMVelocity3d);
    };
};

#endif
