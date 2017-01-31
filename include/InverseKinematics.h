#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "InverseKinematicsV2IPOPT.h"
#include <string>

class InverseKinematics {
    Ipopt::SmartPtr< InverseKinematicsV2IPOPT > solverPointer;
    Ipopt::SmartPtr< Ipopt::IpoptApplication > loader;
    bool updated;
    bool initialized;
    bool alreadyOptimized;
    std::string _parentFrame;
    std::string _targetFrame;
    

    bool update();
    bool getReducedModel(const iDynTree::Model& fullModel, const std::vector< std::string >& consideredJoints, iDynTree::Model& modelOutput);
    bool getFrameIndeces(const std::string& parentFrame, const std::string& endEffectorFrame, const iDynTree::Model model, iDynTree::FrameIndex& parentFrameIndex, iDynTree::FrameIndex& endEffectorFrameIndex);
    void removeUnsupportedJoints(const iDynTree::Model& modelInput, iDynTree::Model& modelOutput);
    bool autoSelectJointsFromTraversal(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, iDynTree::Model& modelOutput);
    
public:
    InverseKinematics();
    
    InverseKinematics(const std::string& solverName);
    
    ~InverseKinematics();

    bool setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame, const std::vector< std::string >& consideredJoints);
    
    bool setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame);
    
    bool setModel(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, const bool autoSelectJoints = true);
    
    void setGains(const iDynTree::Vector3& gains);
    
    void setDesiredPosition(const iDynTree::Vector3& desiredPosition);
    
    void setDesiredQuaternion(const iDynTree::Vector4& desiredQuaternion); 
    
    void setDesiredTransformation(const iDynTree::Transform& p_H_t);
    
    void setDesiredJointPositions(const iDynTree::VectorDynSize& desiredJoints);
    
    bool update(const iDynTree::Vector3& gains, const iDynTree::Vector3 &desiredPosition, const iDynTree::Vector4 &desiredQuaternion, const iDynTree::VectorDynSize &desiredJoints);
    
    signed int runIK(iDynTree::VectorDynSize& jointsOut);

    bool getErrors(iDynTree::Vector3& positionError, iDynTree::Rotation& rotationError, double* angleError);
    
    void getFrames(std::string& parentFrame, std::string& endEffectorFrame);
};
#endif /* end of include guard: INVERSEKINEMATICS_H */
