#include "InverseKinematics.h"


InverseKinematics::InverseKinematics()
:updated(false)
,initialized(false)
,alreadyOptimized(false)
{
    solverPointer = new InverseKinematicsIPOPT();
    loader = IpoptApplicationFactory();
    loader->Options()->SetStringValue("hessian_approximation", "limited-memory");
}

InverseKinematics::~InverseKinematics()
{
}

bool InverseKinematics::prepareProblem(const std::string& filename, const std::vector< std::string >& consideredJoints, const iDynTree::Vector3& gains, const iDynTree::Vector3& desiredPosition, const iDynTree::Vector4& desiredQuaternion, const iDynTree::VectorDynSize& desiredJoints, const std::string& parentFrame, const std::string& endEffectorFrame)
{
    bool success;
    success = solverPointer->loadFromFile(filename, consideredJoints);
    if (success){
        success = solverPointer->setFrames(parentFrame,endEffectorFrame);
        if(success){
            success = solverPointer->update(gains,desiredPosition, desiredQuaternion, desiredJoints);
        }
        else return false;
    }
    else return false;
    
   updated = true;
   
    return true;
}

bool InverseKinematics::setURDF(const std::string& filename, const std::vector< std::string >& consideredJoints)
{
    alreadyOptimized = false;
    return solverPointer->loadFromFile(filename, consideredJoints); 
}

bool InverseKinematics::setURDF(const std::string& filename)
{
    std::vector< std::string > emptyVector;
    emptyVector.clear();
    alreadyOptimized = false;
    return solverPointer->loadFromFile(filename, emptyVector); 
}


bool InverseKinematics::setModel(const iDynTree::Model modelInput)
{
    alreadyOptimized = false;
    return solverPointer->loadFromModel(modelInput);
}

void InverseKinematics::setGains(const iDynTree::Vector3& gains)
{
    solverPointer->gains = gains;
    updated = false;
}

void InverseKinematics::setDesiredPosition(const iDynTree::Vector3& desiredPosition)
{
    solverPointer->desiredPosition = desiredPosition;
    updated = false;
}

void InverseKinematics::setDesiredQuaternion(const iDynTree::Vector4& desiredQuaternion)
{
    solverPointer->desiredQuaternion = desiredQuaternion;
    updated = false;
}

void InverseKinematics::setDesiredTransformation(const iDynTree::Transform p_H_t)
{
    setDesiredPosition(p_H_t.getPosition());
    setDesiredQuaternion(p_H_t.getRotation().asQuaternion());
}

void InverseKinematics::setDesiredJointPositions(const iDynTree::VectorDynSize& desiredJoints)
{
    solverPointer->desiredJoints = desiredJoints;
    updated = false;
}

bool InverseKinematics::setFrames(const std::string& parentFrame, const std::string& endEffectorFrame)
{
    return solverPointer->setFrames(parentFrame, endEffectorFrame);
}


bool InverseKinematics::update(const iDynTree::Vector3& gains, const iDynTree::Vector3& desiredPosition, const iDynTree::Vector4& desiredQuaternion, const iDynTree::VectorDynSize& desiredJoints)
{
    updated = solverPointer->update(gains, desiredPosition, desiredQuaternion, desiredJoints);
    return updated;
}

bool InverseKinematics::update()
{
    updated = solverPointer->update();
    return updated;
}

bool InverseKinematics::getErrors(iDynTree::Vector3& positionError, iDynTree::Rotation& rotationError, double* angleError)
{
    if(!alreadyOptimized)
        return false;
    
    positionError = solverPointer->positionError;
    rotationError = solverPointer->rotationError;
    *angleError = solverPointer->angleError; 
    
    return true;
}


signed int InverseKinematics::runIK(iDynTree::VectorDynSize& jointsOut)
{
    if (!initialized){
        Ipopt::ApplicationReturnStatus status;
        //loader->Options()->SetStringValue("derivative_test", "first-order");
        loader->Options()->SetIntegerValue("print_level", 0);

        status = loader->Initialize();
        if(status != Ipopt::Solve_Succeeded){
            std::cerr<<"[ERROR] Error during IPOPT solver initialization"<< std::endl;
            return -12;
        }
    }
    
    if(!updated){
        bool success;
        success = update();
        if(!success){
            std::cerr << "[ERROR] Error when trying to update IK data" << std::endl;
            return -12;
        }
    }
    
    if(alreadyOptimized){
        loader->ReOptimizeTNLP(solverPointer);
        jointsOut = solverPointer->jointResult;
        return solverPointer->exitCode;
    }
    else {
            std::cerr << "Passing to loader" << std::endl;
    
            loader->OptimizeTNLP(solverPointer);
            alreadyOptimized = true;
            jointsOut = solverPointer->jointResult;
            return solverPointer->exitCode;
    }
}
