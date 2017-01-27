#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "InverseKinematicsIPOPT.h"


class InverseKinematics {
    std::string urdfFileName;
    std::vector< std::string > selectedJoints;
    Ipopt::SmartPtr< InverseKinematicsIPOPT > solverPointer;
    Ipopt::SmartPtr< Ipopt::IpoptApplication > loader;
    bool updated;
    bool initialized;
    bool alreadyOptimized;
    

    bool update();
    
public:
    InverseKinematics();
    
    ~InverseKinematics();
    
    bool prepareProblem(const std::string& filename, 
                        const std::vector< std::string >& consideredJoints, 
                        const iDynTree::Vector3& gains, 
                        const iDynTree::Vector3& desiredPosition, 
                        const iDynTree::Vector4& desiredQuaternion, 
                        const iDynTree::VectorDynSize& desiredJoints, 
                        const std::string& parentFrame, 
                        const std::string& endEffectorFrame);
    
    bool setURDF(const std::string &filename, const std::vector< std::string > &consideredJoints);
    
    bool setURDF(const std::string &filename);
    
    bool setModel(const iDynTree::Model& modelInput);
    
    void setGains(const iDynTree::Vector3& gains);
    
    void setDesiredPosition(const iDynTree::Vector3& desiredPosition);
    
    void setDesiredQuaternion(const iDynTree::Vector4& desiredQuaternion); 
    
    void setDesiredTransformation(const iDynTree::Transform& p_H_t);
    
    void setDesiredJointPositions(const iDynTree::VectorDynSize& desiredJoints);
    
    bool setFrames(const std::string& parentFrame, const std::string& endEffectorFrame);
    
    bool update(const iDynTree::Vector3& gains, const iDynTree::Vector3 &desiredPosition, const iDynTree::Vector4 &desiredQuaternion, const iDynTree::VectorDynSize &desiredJoints);
    
    signed int runIK(iDynTree::VectorDynSize& jointsOut);

    bool getErrors(iDynTree::Vector3& positionError, iDynTree::Rotation& rotationError, double* angleError);
};
#endif /* end of include guard: INVERSEKINEMATICS_H */
