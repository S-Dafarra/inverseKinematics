#include "InverseKinematics.h"
#include "iDynTree/Model/Traversal.h"

InverseKinematics::InverseKinematics()
: updated(false)
, initialized(false)
, alreadyOptimized(false)
{
    solverPointer = new InverseKinematicsIPOPT();
    loader = IpoptApplicationFactory();
    loader->Options()->SetStringValue("hessian_approximation", "limited-memory");
}

InverseKinematics::~InverseKinematics()
{
}

bool InverseKinematics::getReducedModel(const iDynTree::Model& fullModel, const std::vector< std::string >& consideredJoints, iDynTree::Model& modelOutput)
{
    iDynTree::ModelLoader loader;
    bool success;
    
    success = loader.loadReducedModelFromFullModel(fullModel, consideredJoints);

    if (!success){
        std::cerr << "[ERROR] Cannot select joints: " ;
        for (std::vector< std::string >::const_iterator i = consideredJoints.begin(); i != consideredJoints.end(); ++i){
            std::cerr << *i << ' ';
        }
        std::cerr << std::endl;
        return false;
    }
    modelOutput = loader.model();
    
    return true;
}

bool InverseKinematics::getFrameIndeces(const std::string& parentFrame, const std::string& endEffectorFrame, const iDynTree::Model model, iDynTree::FrameIndex& parentFrameIndex, iDynTree::FrameIndex& endEffectorFrameIndex)
{
    parentFrameIndex = model.getFrameIndex(parentFrame);
    endEffectorFrameIndex = model.getFrameIndex(endEffectorFrame);

    if(parentFrameIndex == iDynTree::FRAME_INVALID_INDEX){
        std::cerr<<"[ERROR] Invalid parent frame: "<<parentFrame<< std::endl;
        return false;
    }
    else if(endEffectorFrameIndex == iDynTree::FRAME_INVALID_INDEX){
        std::cerr<<"[ERROR] Invalid End Effector Frame: "<<endEffectorFrame<< std::endl;
        return false;
    }
    
    return true;
}

void InverseKinematics::removeUnsupportedJoints(const iDynTree::Model& modelInput, iDynTree::Model& modelOutput)
{
    iDynTree::ModelLoader loader;
    std::vector< std::string > consideredJoints;
    //int selectedJoints = 0;
    //jointMap.zero();
    
    for(int i=0; i < modelInput.getNrOfJoints(); ++i){
        if(modelInput.getJoint(i)->getNrOfDOFs() == 1){
            
            consideredJoints.push_back(modelInput.getJointName(i));
            
            //jointMap.resize(jointMap.size() + 1);
            //jointMap(selectedJoints) = i;
            //++selectedJoints;
        }
        else std::cerr << "Joint " << modelInput.getJointName(i) << " ignored (" << modelInput.getJoint(i)->getNrOfDOFs() << " DOF)" << std::endl;
    }
    loader.loadReducedModelFromFullModel(modelInput, consideredJoints);
    modelOutput = loader.model();
}


bool InverseKinematics::setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame, const std::vector< std::string >& consideredJoints)
{
    alreadyOptimized = false;

    bool success= false;
    iDynTree::ModelLoader loader;
    iDynTree::FrameIndex parent;
    iDynTree::FrameIndex endEffector;
    iDynTree::Model model;

    success = loader.loadModelFromFile(URDFfilename);
    
    if (!success) {
        std::cerr << "[ERROR] Error loading URDF model from " << URDFfilename << std::endl;
        return false;
    }

    model = loader.model();
        
    if ((!consideredJoints.empty())&&((consideredJoints[0].compare("All") == 0)||(consideredJoints[0].compare("ALL") == 0)||(consideredJoints[0].compare("all") == 0))){
        
        removeUnsupportedJoints(model.copy(),model);
        success = getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false; 
    }
    
    else if (!consideredJoints.empty()){
        
        success = getReducedModel(model, consideredJoints, model);
        if(!success)
            return false;
        
        removeUnsupportedJoints(model.copy(),model);
        
        success = getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false; 
    }
    
    else {
        
        std::cerr << "[IK] Automatically select joints between "<< parentFrame << " and " << endEffectorFrame << std::endl; 
        
        success = getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false; 
        
        std::vector< std::string > consideredJointsAuto;
        iDynTree::Traversal traversal;
        
        model.computeFullTreeTraversal(traversal, model.getFrameLink(parent));
        
        iDynTree::LinkIndex visitedLink = model.getFrameLink(endEffector);
        
        iDynTree::LinkIndex parentLinkIdx;
        iDynTree::IJointConstPtr joint;
        
        while (visitedLink != traversal.getBaseLink()->getIndex())
        {
            parentLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLink)->getIndex();
            joint = traversal.getParentJointFromLinkIndex(visitedLink);
            
            consideredJointsAuto.insert(consideredJointsAuto.begin(), model.getJointName(joint->getIndex()));
            
            visitedLink = parentLinkIdx;
        }
        
        std::cerr << "[IK] Considered joints are:"<< std::endl;
        for (std::vector< std::string >::const_iterator i = consideredJointsAuto.begin(); i != consideredJointsAuto.end(); ++i){
            std::cerr <<"-"<< *i << std::endl;
        }
        std::cerr << std::endl;
        
        success = getReducedModel(model, consideredJointsAuto, model);
        if(!success)
            return false;
        
        removeUnsupportedJoints(model.copy(),model);
        
        success = getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false;
    }

    return solverPointer->loadFromModel(model, parent, endEffector);
}

bool InverseKinematics::setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame)
{
    alreadyOptimized = false;
    
    std::vector< std::string > emptyVector;
    
    emptyVector.resize(0);
    
    return setModelfromURDF(URDFfilename, parentFrame, endEffectorFrame, emptyVector);
} 



bool InverseKinematics::setModel(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame)
{
    iDynTree::FrameIndex parentFrameIndex;
    iDynTree::FrameIndex endEffectorFrameIndex;
    iDynTree::Model model;
    
    removeUnsupportedJoints(modelInput,model);
    
    alreadyOptimized = false;
    if(!getFrameIndeces(parentFrame,endEffectorFrame, model, parentFrameIndex, endEffectorFrameIndex))
        return false;

    return solverPointer->loadFromModel(model, parentFrameIndex, endEffectorFrameIndex);
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

void InverseKinematics::setDesiredTransformation(const iDynTree::Transform& p_H_t)
{
    setDesiredPosition(p_H_t.getPosition());
    setDesiredQuaternion(p_H_t.getRotation().asQuaternion());
}

void InverseKinematics::setDesiredJointPositions(const iDynTree::VectorDynSize& desiredJoints)
{
    solverPointer->desiredJoints = desiredJoints;
    updated = false;
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
    
    solverPointer->computeErrors(positionError,rotationError,angleError);
    
    return true;
}


signed int InverseKinematics::runIK(iDynTree::VectorDynSize& jointsOut)
{
    if (!initialized){
        Ipopt::ApplicationReturnStatus status;
        //loader->Options()->SetStringValue("derivative_test", "first-order");
        loader->Options()->SetIntegerValue("print_level", 0);
        //loader->Options()->SetStringValue("linear_solver", "ma57");

        status = loader->Initialize();
        if(status != Ipopt::Solve_Succeeded){
            std::cerr<<"[ERROR] Error during IPOPT solver initialization"<< std::endl;
            return -6;
        }
        initialized = true;
    }
    
    if(!updated){
        bool success;
        success = update();
        if(!success){
            std::cerr << "[ERROR] Error when trying to update IK data" << std::endl;
            return -6;
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
