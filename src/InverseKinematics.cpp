#include "InverseKinematics.h"


InverseKinematics::InverseKinematics()
:toBeConfigured(true)
,toBeUpdated(true)
{

}

InverseKinematics::InverseKinematics(const std::string& filename, const std::vector< std::string >& consideredJoints, const std::vector< double >& gains, const iDynTree::Vector3& desiredPosition, const iDynTree::Vector4& desiredQuaternion, const iDynTree::VectorDynSize& desiredJoints, const std::string& parentFrame, const std::string& endEffectorFrame)
{
prepareProblem(filename, consideredJoints, gains, desiredPosition, desiredQuaternion, desiredJoints, parentFrame, endEffectorFrame);
}

bool InverseKinematics::prepareProblem(const std::string& filename, const std::vector< std::string >& consideredJoints, const std::vector< double >& gains, const iDynTree::Vector3& desiredPosition, const iDynTree::Vector4& desiredQuaternion, const iDynTree::VectorDynSize& desiredJoints, const std::string& parentFrame, const std::string& endEffectorFrame)
{
    bool success;
    success = solver.load(filename, consideredJoints);
    if (success)
        //success = solver.configure(gains, desiredPosition, desiredQuaternion, desiredJoints, parentFrame, endEffectorFrame);
    //else return false;
    
    if (success){
        toBeConfigured = false;
        toBeUpdated = false;
    }
    else return false;
    
    return true;
}

bool InverseKinematics::prepareProblem()
{

}



signed int InverseKinematics::runIK(iDynTree::VectorDynSize& jointsOut)
{
    Ipopt::SmartPtr<Ipopt::TNLP> solverTempPointer(&solver);
    
    
    solverPointer = solverTempPointer;
}
