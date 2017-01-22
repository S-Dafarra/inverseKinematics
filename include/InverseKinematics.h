#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "InverseKinematicsIPOPT.h"


class InverseKinematics {
    std::string urdfFileName;
    std::vector< std::string > selectedJoints;
    Ipopt::SmartPtr<Ipopt::TNLP> solverPointer;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> loader;
    InverseKinematicsIPOPT solver;
    bool toBeConfigured;
    bool toBeUpdated;
    
public:
    InverseKinematics();
    
    InverseKinematics(const std::string &filename, const std::vector< std::string > &consideredJoints, const std::vector<double>& gains, const iDynTree::Vector3 &desiredPosition, const iDynTree::Vector4 &desiredQuaternion, const iDynTree::VectorDynSize &desiredJoints, const std::string& parentFrame, const std::string& endEffectorFrame);
    
    virtual ~InverseKinematics();
    
    virtual bool prepareProblem(const std::string &filename, const std::vector< std::string > &consideredJoints, const std::vector<double>& gains, const iDynTree::Vector3 &desiredPosition, const iDynTree::Vector4 &desiredQuaternion, const iDynTree::VectorDynSize &desiredJoints, const std::string& parentFrame, const std::string& endEffectorFrame);
    
    virtual bool prepareProblem();
    
    virtual void setURDF(const std::string &filename, const std::vector< std::string > &consideredJoints);
    
    virtual void setGains(const std::vector<double>& gains);
    
    virtual void setDesiredPosition(const iDynTree::Vector3 &desiredPosition);
    
    virtual void setDesiredQuaternion(const iDynTree::Vector4 &desiredQuaternion); 
    
    virtual void setDesiredTransformation(const iDynTree::Transform p_h_t);
    
    virtual void setDesiredJointPositions(const iDynTree::VectorDynSize &desiredJoints);
    
    virtual void setFrames(const std::string& parentFrame, const std::string& endEffectorFrame);
    
    //virtual void setFramesIndexes(const iDynTree::FrameIndex parentFrame, const iDynTree::FrameIndex endEffectorFrame);
    
    virtual signed int runIK(iDynTree::VectorDynSize& jointsOut);

    virtual void getErrors(iDynTree::Vector3& positionError, iDynTree::Rotation& rotationError, double& angleError);
};
#endif /* end of include guard: INVERSEKINEMATICS_H */