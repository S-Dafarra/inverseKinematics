#include "InverseKinematics.h"
#include "URDFdir.h"
#include "iDynTree/Model/Traversal.h"
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <ctime>

int main(int argc, char **argv) {
    
    iDynTree::ModelLoader modelLoader;
    iDynTree::Model model;
    std::string parentFrameName, targetFrameName;
    
    std::cerr<<"Load model from "<<getAbsModelPath("iCubGenova02.urdf")<< std::endl;
    bool ok=modelLoader.loadModelFromFile(getAbsModelPath("iCubGenova02.urdf"));
    model = modelLoader.model();
   
    iDynTree::assertTrue(ok);
    
    std::cerr <<"Model Loaded"<< std::endl;
    
    srand ( clock() );
    
    iDynTree::FrameIndex parentFrame = rand() % model.getNrOfFrames(); //model.getFrameIndex("root_link");
    
    parentFrameName = model.getFrameName(parentFrame);
    std::cerr<<"Selected parent frame "<< parentFrameName << std::endl;
    
    iDynTree::FrameIndex targetFrame = parentFrame;
    
    while(model.getFrameLink(targetFrame) == model.getFrameLink(parentFrame)){
        targetFrame = rand() % model.getNrOfFrames();
    }
    //targetFrame = model.getFrameIndex("r_upper_leg_mtb_acc_11b6");
    targetFrameName = model.getFrameName(targetFrame);
    std::cerr<<"Selected target frame "<< targetFrameName << std::endl;
    
    std::vector< std::string > consideredJoints;
    iDynTree::Traversal traversal;
    
    model.computeFullTreeTraversal(traversal, model.getFrameLink(parentFrame));
    
    iDynTree::LinkIndex visitedLink = model.getFrameLink(targetFrame);
    
    while (visitedLink != traversal.getBaseLink()->getIndex())
    {
        iDynTree::LinkIndex parentLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLink)->getIndex();
        iDynTree::IJointConstPtr joint = traversal.getParentJointFromLinkIndex(visitedLink);
        
        if(joint->getNrOfDOFs() == 1)
            consideredJoints.insert(consideredJoints.begin(), model.getJointName(joint->getIndex()));
        
        visitedLink = parentLinkIdx;
    }
    
    std::cerr << "Considered joints are:"<< std::endl;
    for (std::vector< std::string >::const_iterator i = consideredJoints.begin(); i != consideredJoints.end(); ++i){
        std::cerr <<"-"<< *i << std::endl;
    }
    std::cerr << std::endl;
    ok = modelLoader.loadReducedModelFromFullModel(model.copy(),consideredJoints);
    
    iDynTree::assertTrue(ok);
    
    std::cerr<<"Reduced Model Loaded" << std::endl;
    
    model = modelLoader.model();
    
    std::vector< std::pair<double, double> > jointsLimits;
    
    jointsLimits.clear();
    jointsLimits.resize(model.getNrOfDOFs());
    for (iDynTree::JointIndex j = 0; j < model.getNrOfJoints(); j++) {
            std::pair<double, double> limits; //first is min, second max
            iDynTree::IJointPtr joint;
            joint = model.getJoint(j);
            if(joint->hasPosLimits()){
                joint->getPosLimits(0,limits.first,limits.second); //no joints with two DOF can be considered. Deal with it accordingly in the URDF
            }
            else {
                  limits.first = -2*M_PI;
                  limits.second = 2*M_PI;
            }
            
            if( joint->getNrOfDOFs() == 1 ) 
            {
                jointsLimits[joint->getDOFsOffset()] = limits;  
            }
    }
    
    
    std::cerr << "Joints Limits:" << std::endl;
    for(int i = 0; i < jointsLimits.size(); ++i){
        std::cerr<< jointsLimits[i].first<<" < "<< model.getJointName(i) << " < " << jointsLimits[i].second << " ";
    }
    std::cerr << std::endl;
    
    //////////////////////////////////////
    /* Calling Inverse Kinematcs solver */
    //////////////////////////////////////
    
    InverseKinematics solver;
    
    std::cerr<<"Created solver object"<<std::endl;
    
    ok = solver.setModel(model, parentFrameName, targetFrameName);
    //ok = solver.setModelfromURDF(getAbsModelPath("iCubGenova02.urdf"), parentFrameName, targetFrameName);
    //ok = solver.setModelfromURDF(getAbsModelPath("iCubGenova02.urdf"), parentFrameName, targetFrameName, consideredJoints);
    
    iDynTree::assertTrue(ok);
    
    std::cerr << "Model Set" <<std::endl;
    
    iDynTree::Vector3 gains;
    gains(0) = 100;
    gains(1) = 10;
    gains(2) = 0.01;
    
    solver.setGains(gains);
    
    std::cerr << "Gains set" << std::endl;
    
    iDynTree::VectorDynSize desJoints(model.getNrOfDOFs());
    desJoints.zero();
    
    solver.setDesiredJointPositions(desJoints);
    
    std::cerr << "Desired Joints set" << std::endl;
    
    iDynTree::KinDynComputations iKDC;
    iDynTree::assertTrue(iKDC.loadRobotModel(model));
    
    iDynTree::VectorDynSize jointsTest(model.getNrOfDOFs());
    signed int exitCode;
    iDynTree::VectorDynSize jointsIK;
    iDynTree::Vector3 positionErrorComputed, positionErrorIK;
    iDynTree::Rotation rotationErrorIK, rotationErrorComputed;
    double angleError;
    clock_t now;
    double elapsed_time;
    
    for(int j=0; j<4; ++j){
    
        srand ( clock() );
        
        std::cerr << "The target joints are:" << std::endl;
        
        for(int i=0; i < jointsTest.size(); ++i){
            jointsTest(i) = jointsLimits[i].first + ( (double) rand() / RAND_MAX )*(jointsLimits[i].second - jointsLimits[i].first);
            std::cerr << "$ " << jointsTest(i) << std::endl;
        }
        
        iKDC.setJointPos(jointsTest);
        
        iDynTree::Transform p_H_t = iKDC.getRelativeTransform(model.getFrameIndex(parentFrameName),model.getFrameIndex(targetFrameName));
        
        iKDC.getJointPos(jointsTest);
        
        std::cerr << "iKDC joints: " <<  jointsTest.toString() << std::endl;
        
        solver.setDesiredTransformation(p_H_t);
        
        std::cerr << "Desired Transformation set" << std::endl;
        
        
        now = clock();
        exitCode = solver.runIK(jointsIK);
        elapsed_time = clock() - now;
        elapsed_time = elapsed_time/CLOCKS_PER_SEC;
        std::cerr << "Elapsed Time: "<< elapsed_time << std::endl;
        
        std::cerr << "Exit Code " << exitCode << std::endl;
        
        iDynTree::assertTrue((exitCode >= 0));
        
        std::cerr << "Joints IK:" << jointsIK.toString() << std::endl;
       
        iDynTree::assertTrue(iKDC.setJointPos(jointsIK));
        
        iDynTree::toEigen(positionErrorComputed) = iDynTree::toEigen(p_H_t.getPosition()) - iDynTree::toEigen( iKDC.getRelativeTransform(model.getFrameIndex(parentFrameName),model.getFrameIndex(targetFrameName)).getPosition() );
        rotationErrorComputed = p_H_t.getRotation()*(iKDC.getRelativeTransform(model.getFrameIndex(parentFrameName),model.getFrameIndex(targetFrameName)).getRotation().inverse());
        
        std::cerr << "Computed position error: " << positionErrorComputed.toString() << std::endl;
        std::cerr << "Computed rotation error: " << std::endl << rotationErrorComputed.toString() << std::endl;
        
        std::cerr << "Retrieving errors from solver" << std::endl;
        solver.getErrors(positionErrorIK, rotationErrorIK, &angleError);
        
        std::cerr << "Retrieved position error: " << positionErrorIK.toString() << std::endl;
        std::cerr << "Retreived rotation error: "<< std::endl << rotationErrorIK.toString() << std::endl;
        std::cerr << "Retreived angle error: "<< angleError << std::endl;
        
        
    }

}
