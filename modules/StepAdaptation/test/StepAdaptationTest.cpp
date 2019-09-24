/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Utils.h>
#include <Eigen/Core>
#include "iDynTree/Core/EigenHelpers.h"
#include "iDynTree/Core/MatrixDynSize.h"
#include <cmath>
#include <memory>
#include <iostream>
#include <ctime>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include<StepAdaptator.hpp>
#include<DCMPlanning.hpp>
#include <yarp/os/RpcClient.h>
#include <WalkingLogger.hpp>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>



int main(int argc, char **argv) {

    std::unique_ptr<WalkingLogger> m_walkingLogger; /**< Pointer to the Walking Logger object. */

    // prepare and configure the resource finder
    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }

    std::unique_ptr<StepAdaptator> m_stepAdaptator;
    std::unique_ptr<DCMPlanning> m_DCMPlanner;
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("dcm_walking_with_joypad.ini");
    rf.configure(argc, argv);
    bool m_dumpData;
    m_dumpData = rf.check("dump_data", yarp::os::Value(false)).asBool();
    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    double  m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.016)).asDouble();
    double omega;
    // m_walkingLogger->startRecord({"record","dist","steplength","steptime","dcmoffset"});
    // initialize the step adaptation
    omega = sqrt(9.81 / 0.53);
    yarp::os::Bottle& stepAdaptatorOptions = rf.findGroup("STEP_ADAPTATOR");
    stepAdaptatorOptions.append(generalOptions);
    m_DCMPlanner = std::make_unique<DCMPlanning>(omega);
    m_stepAdaptator = std::make_unique<StepAdaptator>();
    m_stepAdaptator->initialize(stepAdaptatorOptions);
    //    if(!)
    //    {
    //        yError() << "[configure] Unable to initialize the step adaptator!";
    //    }
    iDynTree::Vector2 delta;
    if(!YarpHelper::getVectorFromSearchable(stepAdaptatorOptions, "delta",  delta))
    {
        yError() << "[StepAdaptator::initialize] Unable to get the vector delta";
        return false;
    }
    if(m_dumpData)
    {

        m_walkingLogger = std::make_unique<WalkingLogger>();
        yarp::os::Bottle& loggerOptions = rf.findGroup("WALKING_LOGGER");
        if(!m_walkingLogger->configure(loggerOptions, "stepAdaptation"))
        {
            yError() << "[configure] Unable to configure the logger.";
            return false;
        }
        m_walkingLogger->startRecord({"record","foot_adapted_x","foot_adapted_y","currentDCMx","currentDCMy","currentZMPx","currentZMPy","currentCoMx","currentCoMy","dcm_offset_x","dcm_offset_y","timeX","timeY","itimeX","itimeY"});

    }


    //    m_stepAdaptator = std::make_unique<StepAdaptator>();


    iDynTree::VectorFixSize<5> nominalValues;
    iDynTree::Vector3 currentValues;

    double c;
    //double omega=sqrt(9.81/0.6);
    int i=0;
    double nomStepTiming=0.5;
    double adaptedStepTiming=0.5;
    double stepTiming1=0.5;
    double sigma;
    double nextStepPositionx=0.5;
    double nomStepLength=0.5;
    double numberOfStep=5;
    double stepWidth=0.1;
    double nominalDCMOffset;
    iDynTree::Vector2 timed;
    timed(0)=0;


    double nomStepWidth=0;
    double lengthOfPelvis=0.1;

    //iDynTree::Vector3 leftAdaptedStepParameters;
    double alpha=0;
    double initDCM=0;
    double initTimining=0;
    double initiDCMOffset=0;
    double initStepPosition1=0;
    double a;
    double b;
    double comHeight=0.8;



    iDynTree::Vector4 tempp;
    double dt=0.0100000000;
    double Pt=0;
    iDynTree::Vector2 CoeffA;
    iDynTree::Vector2 CoeffB;
    iDynTree::Vector2 finalZMP;
    iDynTree::Vector2 initialZMP;
    iDynTree::Vector2 currentZMP;
        iDynTree::Vector2 currentDCMOffset;
    iDynTree::Vector2 currentDCM;
    iDynTree::Vector2 currentCoM;
    iDynTree::Vector2 middleZMP;
    iDynTree::Vector2 stepLW;
    bool isleft;

    iDynTree::Vector2 nextZMP;
    iDynTree::Vector2 adaptedNextZMP;
    iDynTree::Vector2 adaptedNextDCMOffset;
    adaptedNextDCMOffset.zero();
    initialZMP.zero();
    finalZMP.zero();
    iDynTree::Vector2 m_nominalDCMOffset;

    // iDynTree::Vector2 adaptedNextZMP;
    double landa;
    iDynTree::toEigen( middleZMP)=(iDynTree::toEigen(finalZMP)/2+iDynTree::toEigen(initialZMP))/(2);
    m_nominalDCMOffset(0)=nomStepLength/(exp(omega*stepTiming1)-1);
    currentDCM(0)=nomStepLength/(exp(omega*stepTiming1)-1);
    currentDCM(1)=0;//nom/(exp(omega*nomStepLength)-1);
    currentZMP.zero();
    m_nominalDCMOffset(1)=pow(-1,1)*(lengthOfPelvis/(1+exp(omega*stepTiming1)));
    iDynTree::toEigen(currentDCM)=iDynTree::toEigen(currentZMP)+iDynTree::toEigen(m_nominalDCMOffset);

    currentCoM=currentDCM;
    iDynTree::toEigen(finalZMP)=iDynTree::toEigen(initialZMP)+iDynTree::toEigen(delta);
    for (int i=1;i<=numberOfStep;i++) {
        adaptedStepTiming=nomStepTiming;
        int adaptedStepTimingIndex=adaptedStepTiming/dt;
        stepLW(0)=nomStepLength;
        stepLW(1)=pow(-1,i)*stepWidth;
//        yInfo()<<i<<i<<i<<i<<i<<"iii";

        if (int(pow(-1,i))==1) {
            isleft=true;
//            yInfo()<<"injaaahshhggdggdggsggsgggs"<<2;
//            yInfo()<<"injaaahshhggdggdggsggsgggs"<<5544;
//            yInfo()<<"injaaahshhggdggdggsggsgggs"<<245544;
//            yInfo()<<"injaaahshhggdggdggsggsgggs"<<5444455;
            m_nominalDCMOffset(1)=pow(-1,i)*(lengthOfPelvis/(1+exp(omega*adaptedStepTiming)));
        }
        else {
            isleft=false;
            m_nominalDCMOffset(1)=pow(-1,i)*(lengthOfPelvis/(1+exp(omega*adaptedStepTiming)));
        }
        if (i!=1) {
            iDynTree::toEigen(currentDCM)=iDynTree::toEigen(currentZMP)+iDynTree::toEigen(m_nominalDCMOffset);
        }
        for(int index=0;index<adaptedStepTimingIndex;index++){
            adaptedStepTiming=adaptedStepTiming-dt;
            timed(0)=timed(0)+dt;
//            yInfo()<<index<<index<<index<<index<<index<<"index";
            //time=0;

            m_stepAdaptator->setTimings(omega,0,adaptedStepTiming);
            landa=exp(-omega*adaptedStepTiming);


            m_stepAdaptator->setCurrentZmpPosition(currentZMP);
            iDynTree::toEigen( CoeffA)=(iDynTree::toEigen(finalZMP)-iDynTree::toEigen(currentZMP))/(landa-1);
            iDynTree::toEigen( CoeffB)=(landa*iDynTree::toEigen(currentZMP)-iDynTree::toEigen(finalZMP))/(landa-1);
            iDynTree::toEigen( nextZMP)=(iDynTree::toEigen(currentZMP)+iDynTree::toEigen(stepLW));
            m_stepAdaptator->setCurrentDcmPosition(currentDCM);
            m_stepAdaptator->setNominalNextStepPosition(nextZMP,0.0000);
            m_stepAdaptator->setCurrentZmpPosition(currentZMP);
            m_stepAdaptator->setNominalDcmOffset(m_nominalDCMOffset);
            m_stepAdaptator->setFinalZMPPosition(finalZMP);


            m_stepAdaptator->solve(isleft);

            adaptedNextZMP=m_stepAdaptator->getDesiredZmp();
            adaptedStepTiming=m_stepAdaptator->getDesiredImpactTime();
            adaptedNextDCMOffset=m_stepAdaptator->getDesiredNextDCMOffset();
//            yInfo()<<currentZMP(0)<<finalZMP(0)<<nextZMP(0)<<currentZMP(1)<<finalZMP(1)<<nextZMP(1);
            m_DCMPlanner->setBoundryZMPPosition(currentZMP,finalZMP);
            m_DCMPlanner->setInitialDCMPosition(currentDCM);
            m_DCMPlanner->solveDCMDynamics(dt);
            m_DCMPlanner->getDCMPosition(currentDCM);

            m_DCMPlanner->solveCoMDynamics(dt,currentCoM,currentDCM);
            m_DCMPlanner->getCoMPosition(currentCoM);


            yarp::os::Time::delay(0.017);
           // iDynTree::toEigen(currentZMP)=(iDynTree::toEigen(CoeffA))*exp(-omega*dt)+iDynTree::toEigen(CoeffB);
            iDynTree::Vector2 timingTemp;
            timingTemp(0)=adaptedStepTiming;
            if(m_dumpData)
            {
            m_walkingLogger->sendData(adaptedNextZMP,currentDCM,currentZMP,currentCoM,adaptedNextDCMOffset,timingTemp,timed);
            }
            //adaptedStepTiming=adaptedStepTiming-0.01;
            iDynTree::toEigen(currentZMP)=(iDynTree::toEigen(CoeffA))*exp(-omega*dt)+iDynTree::toEigen(CoeffB);

            //adaptedStepTiming=adaptedStepTiming-0.01;

        }
        initialZMP=adaptedNextZMP;
        iDynTree::toEigen(finalZMP)=iDynTree::toEigen(adaptedNextZMP)+iDynTree::toEigen(delta);
        iDynTree::toEigen(currentZMP)=iDynTree::toEigen(adaptedNextZMP)-iDynTree::toEigen(delta)/2;

    }
    //for (int k=1;k<12;k++){
    //    alpha=alpha+0.011;
    //    alpha=0;
    //    a=0;
    //    b=0.5/(exp(omega*(stepTiming1-0.01))-1);;
    //    i=0;
    //    double kk=0;
    //    nextStepPosition=0.5;
    //    iDynTree::Vector3 leftAdaptedStepParameters;
    //    double RStepTiming=stepTiming;

    //    for (int var2=1;var2<=6;var2++) {
    //        yInfo()<<var2<<var2;
    //        RStepTiming=stepTiming1;


    //        for(int var=1;var<=int(((RStepTiming+(stepTiming1-stepTiming))/0.0100)+0.001);var++){
    //            i++;
    //            stepTiming=stepTiming-0.01;
    //            sigma=exp(omega*stepTiming);

    //            //nomStepTiming=(jLeftstepList.at(1).impactTime-jRightstepList.at(0).impactTime)/(1+switchOverSwingRatio);
    //            nominalDCMOffset=0.5/(exp(omega*stepTiming1)-1);

    //            nominalValues(0)=nextStepPosition;
    //            nominalValues(1)=sigma;
    //            nominalValues(2)=nominalDCMOffset;
    //            nominalValues(3)=0;
    //            nominalValues(4)=omega;


    //            currentValues(0)=a;
    //            currentValues(1)=b;
    //            currentValues(2)=0;





    //            if (((var+1)==(int(((RStepTiming+(stepTiming1-stepTiming))/0.0100)+0.001)))) {
    //                kk=kk+1;
    //                //a=b-0.1;
    //                a=a+0.50;
    //                b=a+nominalDCMOffset;
    //                stepTiming=0.50;

    //                nextStepPosition=nextStepPosition+0.5;
    //                yInfo()<<"milad"<<"ddjdjjjdjj"<<kk;
    //            }

    //            timed(0)=timed(0)+0.0100;

    //            if(!m_stepAdaptator->initialize(stepAdaptatorOptions))
    //            {
    //                yError() << "[configure] Unable to initialize the step adaptator!";
    //                return false;
    //            }



    //            if(!m_stepAdaptator->RunStepAdaptator(nominalValues,currentValues))
    //            {
    //                yError() << "[updateModule] Unable to solve the QP problem of step adaptation.";
    //                return false;
    //            }


    //            if(!m_stepAdaptator->solve())
    //            {
    //                yError() << "[updateModule] Unable to solve the QP problem of step adaptation.";
    //                return false;
    //            }

    //            if(!m_stepAdaptator->getControllerOutput(leftAdaptedStepParameters))
    //            {
    //                yError() << "[updateModule] Unable to get the step adaptation output.";
    //                return false;
    //            }
    //            double mil=leftAdaptedStepParameters(0)+leftAdaptedStepParameters(2)+(currentValues(0)-currentValues(1))*(leftAdaptedStepParameters(1))-currentValues(0);
    //            RStepTiming=((log(leftAdaptedStepParameters(1)))/omega);

    //            tempp(0)=(b-a);/*mil;*/
    //            tempp(1)=(currentValues(0)-currentValues(1));
    //            tempp(2)=leftAdaptedStepParameters(1);
    //            tempp(3)=currentValues(0);
    //            //        if(i==720){
    //            //            initDCM=b;
    //            //            initStepPosition1=leftAdaptedStepParameters(0);
    //            //            initTimining=leftAdaptedStepParameters(1);
    //            //            initiDCMOffset=leftAdaptedStepParameters(2);
    //            //        }

    //            //        if(i==726){
    //            //        tempp(0)=b-initDCM;
    //            //        tempp(1)=leftAdaptedStepParameters(0)-initStepPosition1;
    //            //        tempp(2)=leftAdaptedStepParameters(1)-initTimining;
    //            //        tempp(3)=leftAdaptedStepParameters(2)-initiDCMOffset;
    //            //        yInfo()<<leftAdaptedStepParameters(0)<<initStepPosition1;
    //            //        }
    //            b =(b-a)*exp(omega*0.01)+a;
    //            if(i==112){
    //                b=b+1*0.18;
    //            }

    //            m_walkingLogger->sendData(leftAdaptedStepParameters,timed,nominalValues,tempp);


    //            yInfo()<<RStepTiming<<i<<var<<int(((RStepTiming+(stepTiming1-stepTiming))/0.0100)+0.001);


    //            yarp::os::Time::delay(0.01);
    //        }
    //    }
    //       m_walkingLogger->sendData(tempp);
    //}
    if(m_dumpData)
        m_walkingLogger->quit();

    return EXIT_SUCCESS;
}