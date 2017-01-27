#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "InverseKinematicsIPOPT.h"


class InverseKinematics {
    std::string urdfFileName;
    std::vector< std::string > selectedJoints;
    Ipopt::SmartPtr<InverseKinematicsIPOPT> solverPointer;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> loader;
    bool updated;
    bool initialized;
    bool alreadyOptimized;
    

    virtual bool update();
    
public:
    InverseKinematics();
    
    virtual ~InverseKinematics();
    
    virtual bool prepareProblem(const std::string& filename, const std::vector< std::string >& consideredJoints, const iDynTree::Vector3& gains, const iDynTree::Vector3& desiredPosition, const iDynTree::Vector4& desiredQuaternion, const iDynTree::VectorDynSize& desiredJoints, const std::string& parentFrame, const std::string& endEffectorFrame);
    
    virtual bool setURDF(const std::string &filename, const std::vector< std::string > &consideredJoints);
    
    virtual bool setURDF(const std::string &filename);
    
    virtual bool setModel(const iDynTree::Model modelInput);
    
    virtual void setGains(const iDynTree::Vector3& gains);
    
    virtual void setDesiredPosition(const iDynTree::Vector3 &desiredPosition);
    
    virtual void setDesiredQuaternion(const iDynTree::Vector4 &desiredQuaternion); 
    
    virtual void setDesiredTransformation(const iDynTree::Transform p_H_t);
    
    virtual void setDesiredJointPositions(const iDynTree::VectorDynSize &desiredJoints);
    
    virtual bool setFrames(const std::string& parentFrame, const std::string& endEffectorFrame);
    
    virtual bool update(const iDynTree::Vector3& gains, const iDynTree::Vector3 &desiredPosition, const iDynTree::Vector4 &desiredQuaternion, const iDynTree::VectorDynSize &desiredJoints);
    
    virtual signed int runIK(iDynTree::VectorDynSize& jointsOut);

    virtual bool getErrors(iDynTree::Vector3& positionError, iDynTree::Rotation& rotationError, double* angleError);
};
#endif /* end of include guard: INVERSEKINEMATICS_H */