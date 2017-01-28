#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "InverseKinematicsIPOPT.h"


class InverseKinematics {
    Ipopt::SmartPtr< InverseKinematicsIPOPT > solverPointer;
    Ipopt::SmartPtr< Ipopt::IpoptApplication > loader;
    bool updated;
    bool initialized;
    bool alreadyOptimized;
    

    bool update();
    bool getReducedModel(const iDynTree::Model& fullModel, const std::vector< std::string >& consideredJoints, iDynTree::Model& modelOutput);
    bool getFrameIndeces(const std::string& parentFrame, const std::string& endEffectorFrame, const iDynTree::Model model, iDynTree::FrameIndex& parentFrameIndex, iDynTree::FrameIndex& endEffectorFrameIndex);
    void removeUnsupportedJoints(const iDynTree::Model& modelInput, iDynTree::Model& modelOutput);
    
public:
    InverseKinematics();
    
    ~InverseKinematics();

    bool setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame, const std::vector< std::string >& consideredJoints);
    
    bool setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame);
    
    bool setModel(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame);
    
    void setGains(const iDynTree::Vector3& gains);
    
    void setDesiredPosition(const iDynTree::Vector3& desiredPosition);
    
    void setDesiredQuaternion(const iDynTree::Vector4& desiredQuaternion); 
    
    void setDesiredTransformation(const iDynTree::Transform& p_H_t);
    
    void setDesiredJointPositions(const iDynTree::VectorDynSize& desiredJoints);
    
    bool update(const iDynTree::Vector3& gains, const iDynTree::Vector3 &desiredPosition, const iDynTree::Vector4 &desiredQuaternion, const iDynTree::VectorDynSize &desiredJoints);
    
    signed int runIK(iDynTree::VectorDynSize& jointsOut);

    bool getErrors(iDynTree::Vector3& positionError, iDynTree::Rotation& rotationError, double* angleError);
};
#endif /* end of include guard: INVERSEKINEMATICS_H */
