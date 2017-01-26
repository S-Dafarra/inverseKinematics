#include "InverseKinematicsIPOPT.h"

#include <cassert>

using namespace std;
using namespace iDynTree;
using namespace Ipopt;

InverseKinematicsIPOPT::InverseKinematicsIPOPT()
: exitCode(-12)
, parentFrame(0)
, endEffectorFrame(0)
,modelLoaded(false)
,framesLoaded(false)
,gainsLoaded(false)
,angleError(0)
{
    positionError.zero();
    rotationError.Identity();
}

InverseKinematicsIPOPT::~InverseKinematicsIPOPT()
{}

void InverseKinematicsIPOPT::removeJoints(const Model modelInput)
{
    ModelLoader loader;
    std::vector< std::string > consideredJoints;
    int selectedJoints = 0;
    jointMap.zero();
    
    for(int i=0; i < modelInput.getNrOfJoints(); ++i){
        if(modelInput.getJoint(i)->getNrOfDOFs() == 1){
            consideredJoints.reserve(1);
            consideredJoints.push_back(modelInput.getJointName(i));
            
            jointMap.resize(jointMap.size() + 1);
            jointMap(selectedJoints) = i;
            ++selectedJoints;
        }
        else std::cerr << "Joint " << modelInput.getJointName(i) << " ignored (" << modelInput.getJoint(i)->getNrOfDOFs() << " DOF)" << std::endl;
    }
    loader.loadReducedModelFromFullModel(modelInput, consideredJoints);
    model = loader.model();
}


bool InverseKinematicsIPOPT::loadFromModel(const Model modelInput)
{
    removeJoints(modelInput);
    iKDC.loadRobotModel(model);
    
    jointsLimits.reserve(model.getNrOfDOFs());
    
    for (iDynTree::JointIndex j = 0; j < model.getNrOfDOFs(); j++) {
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
            jointsLimits.push_back(limits);
        }
    
    std::cerr << "Joints Limits:" << std::endl;
    for(int i = 0; i < jointsLimits.size(); ++i){
        std::cerr<< jointsLimits[i].first<<" < "<< model.getJointName(i) << " < " << jointsLimits[i].second << " ";
    }
    std::cerr << std::endl;
    
    desiredJoints.resize(model.getNrOfDOFs());
    
    jointResult.resize(model.getNrOfDOFs());
    
    totalDOF =7 + model.getNrOfDOFs();
    
    hessian.resize(totalDOF,totalDOF);
    gradient.resize(totalDOF);
    
    Ep.resize(3,totalDOF);
    Ep.zero();
    toEigen(Ep).block<3,3>(0,0).setIdentity();
    
    Eq.resize(4,totalDOF);
    Eq.zero();
    toEigen(Eq).block<4,4>(0,3).setIdentity();
    
    Edof.resize(model.getNrOfDOFs(),totalDOF);
    Edof.zero();
    toEigen(Edof).block(0,7,model.getNrOfDOFs(),model.getNrOfDOFs()).setIdentity();
    
    modelLoaded = true;
    return true;
}


bool InverseKinematicsIPOPT::loadFromFile(const string& filename, const vector< string >& consideredJoints)
{
    bool success= false;
    ModelLoader loader;

    success = loader.loadModelFromFile(filename);
    
    if (!success) {
        std::cerr << "[ERROR] Error loading URDF model from " << filename << std::endl;
        return false;
    }

    model = loader.model();
    
    const Model input=model;
    
    if (!consideredJoints.empty())
        success = loader.loadReducedModelFromFullModel(input, consideredJoints);
    
    if (!success){
        std::cerr << "[ERROR] Cannot select joints: " ;
        for (std::vector< string >::const_iterator i = consideredJoints.begin(); i != consideredJoints.end(); ++i)
            std::cerr << *i << ' ';
        std::cerr << std::endl;
        return false;
    }
    
    model = loader.model();
    
    return loadFromModel(model);
}

bool InverseKinematicsIPOPT::setFrames(const string& parentFrameIn, const string& endEffectorFrameIn)
{
   if(!modelLoaded){
       std::cerr<<"[ERROR] First you have to load a model"<< std::endl;
       return false;
   }
    parentFrame = model.getFrameIndex(parentFrameIn);
    endEffectorFrame = model.getFrameIndex(endEffectorFrameIn);
    
    if(parentFrame == FRAME_INVALID_INDEX){
        std::cerr<<"[ERROR] Invalid parent frame: "<<parentFrameIn<< std::endl;
        return false;}
    else if(endEffectorFrame == FRAME_INVALID_INDEX){
        std::cerr<<"[ERROR] Invalid End Effector Frame: "<<endEffectorFrameIn<< std::endl;
        return false;}
    
    framesLoaded = true;
    return true;
}


bool InverseKinematicsIPOPT::update(const Vector3& gainsIn, const Vector3& desiredPositionIn, const Vector4& desiredQuaternionIn, const VectorDynSize& desiredJointsIn)
{
    if(!modelLoaded){
       std::cerr<<"[ERROR] First you have to load a model"<< std::endl;
       return false;
    }

    if(desiredJointsIn.size() != (totalDOF-7)){
        std::cerr<<"[ERROR] Dimension of desired joints vector lower than the number of considered joints"<<desiredJointsIn.size()<<"!="<<model.getNrOfDOFs()<< std::endl;
        return false;
    }
    
    toEigen(hessian) =  gainsIn(0)*toEigen(Ep).transpose()*toEigen(Ep) + gainsIn(1)*toEigen(Eq).transpose()*toEigen(Eq) + gainsIn(2)*toEigen(Edof).transpose()*toEigen(Edof);
    toEigen(gradient) = -(gainsIn(0)*toEigen(desiredPositionIn).transpose()*toEigen(Ep) + gainsIn(1)*toEigen(desiredQuaternionIn).transpose()*toEigen(Eq) + gainsIn(2)*toEigen(desiredJointsIn).transpose()*toEigen(Edof));
    
    desiredPosition = desiredPositionIn;
    desiredJoints = desiredJointsIn;
    desiredQuaternion = desiredQuaternionIn;
    gains = gainsIn;
    
    gainsLoaded = true;
    return true;
}

bool InverseKinematicsIPOPT::update()
{
    return update(gains,desiredPosition,desiredQuaternion,desiredJoints);

}

void InverseKinematicsIPOPT::twistToQuaternionTwist(Vector4& quaternion, MatrixFixSize< 7, 6 >& mapOut)
{
    MatrixFixSize<7,6> map;
    MatrixFixSize<3,4> omegaToQuat;
    Eigen::Vector3d imQuat;
    IndexRange rowRange, columnRange;
    
    imQuat = toEigen(quaternion).tail(3);
    
    omegaToQuat.zero();
    
    toEigen(omegaToQuat).leftCols<1>() = -imQuat;
    toEigen(omegaToQuat).block<3,3>(0,1).setIdentity();
    toEigen(omegaToQuat).block<3,3>(0,1) *= quaternion(0);
    toEigen(omegaToQuat).block<3,3>(0,1) += skew(imQuat);
    toEigen(omegaToQuat) *= 0.5;
    
    map.zero();
    
    rowRange.offset = 0;
    rowRange.size = 3;
    columnRange.offset = 0;
    columnRange.size = 3;
    
    setSubMatrixToIdentity(map,rowRange,columnRange);
    
    rowRange.offset = 3;
    rowRange.size = 4;
    columnRange.offset = 3;
    columnRange.size = 3;
    setSubMatrix(map,rowRange,columnRange,toEigen(omegaToQuat).transpose());

    mapOut = map;
}

void InverseKinematicsIPOPT::relativeJacobian(const VectorDynSize& configuration, MatrixDynSize& jacobianOut)
{
    Matrix6x6 right2mixed, left2mixed;
    FrameFreeFloatingJacobian e_J_we_temp, p_J_wp_temp;
    MatrixDynSize e_J_we, p_J_wp;
    
    
    iKDC.setJointPos(configuration);
    iKDC.setFrameVelocityRepresentation(BODY_FIXED_REPRESENTATION); //left trivialized velocity
    
    left2mixed = iKDC.getRelativeTransformExplicit(endEffectorFrame,parentFrame,endEffectorFrame,endEffectorFrame).asAdjointTransform(); //is the adjoint transformation from left-trivialized velocity to mixed velocity with the origin on the target frame and the orientation of the parent frame
    right2mixed = iKDC.getRelativeTransformExplicit(endEffectorFrame,parentFrame,parentFrame,parentFrame).asAdjointTransform(); //is the adjoit trasnformation from right-trivialized velocity to mixed velocity. It comes from a multiplication of two adjoints: from left to mixed times from right to left
    
    p_J_wp_temp.resize(iKDC.model());
    e_J_we_temp.resize(iKDC.model());
    iKDC.getFrameFreeFloatingJacobian(parentFrame, p_J_wp_temp); //getting the jacobian from world to parent frame with left-trivialized velocity representation
    iKDC.getFrameFreeFloatingJacobian(endEffectorFrame, e_J_we_temp);  //getting the jacobian from world to target frame with left-trivialized velocity representation
    
    e_J_we.resize(6,totalDOF-7);
    p_J_wp.resize(6,totalDOF-7);
    
    toEigen(e_J_we) = toEigen(e_J_we_temp).rightCols(totalDOF-7); //removing the base contribution
    toEigen(p_J_wp) = toEigen(p_J_wp_temp).rightCols(totalDOF-7);
    
    jacobianOut.resize(6,totalDOF-7);
    jacobianOut.zero();
    toEigen(jacobianOut) = toEigen(left2mixed)*toEigen(e_J_we) - toEigen(right2mixed)*toEigen(p_J_wp);
}


bool InverseKinematicsIPOPT::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    if(!modelLoaded){
        std::cerr<<"[ERROR] First you have to load the model"<< std::endl;
        return false;
    }
    else if(!framesLoaded){
        std::cerr<<"[ERROR] First you have to select the frames"<< std::endl;
        return false;
    }
    else if(!gainsLoaded){
        std::cerr<<"[ERROR] First you have to define cost function gains"<< std::endl;
        return false;
    }
    
    n = totalDOF; //position and orientation of the end effector (3 for position and 4 for the quaternion of the orientation) + joints'DoF, in this order.
    m = 8; //7 for forward kinematics (relates the 7 dofs of the end effector with the joint dofs) + 1 on the norm of the quaternion.
    
    nnz_jac_g = 7 + (totalDOF-7)*7 + 4;
    nnz_h_lag = n*n; //dense
    
    index_style = Ipopt::TNLP::C_STYLE;
    
    return true;
}

bool InverseKinematicsIPOPT::get_bounds_info(Ipopt::Index n, Number* x_l, Number* x_u, Ipopt::Index m, Number* g_l, Number* g_u)
{
    assert((n == totalDOF) && (m==8));
    for (Ipopt::Index i = 0; i < 3; ++i) {
        x_l[i] = -2e+19;
        x_u[i] =  2e+19;
    }

    x_l[3] =  0; x_u[3] = 1;
    x_l[4] = -1; x_u[4] = 1;
    x_l[5] = -1; x_u[5] = 1;
    x_l[6] = -1; x_u[6] = 1;
    
    for (Ipopt::Index j = 7; j < totalDOF; j++){
        x_l[j] = jointsLimits[j-7].first; x_u[j] = jointsLimits[j-7].second; 
    }
    
    for (Ipopt::Index c = 0; c < 7; c++){
        g_l[c] = g_u[c] = 0; //forward kinematics equality constriants
    }
    g_l[7] = g_u[7] = 1; //equality constraint on the modulus of the quaternion
    
    return true;
}

bool InverseKinematicsIPOPT::get_starting_point(Ipopt::Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Ipopt::Index m, bool init_lambda, Number* lambda)
{
    if(init_z) return false;
    if(init_lambda) return false;

    Transform p_H_e; //from parent to end effector
    Eigen::Map< Eigen::VectorXd > x_e (x, 7);
 
    if(init_x){
        iKDC.setJointPos(desiredJoints);
        p_H_e = iKDC.getRelativeTransform(parentFrame,endEffectorFrame);
        x_e.head<3>() = toEigen(p_H_e.getPosition());
        x_e.tail<4>() = toEigen(p_H_e.getRotation().asQuaternion());
        
        for(int i = 7; i < totalDOF; i++){
            x[i] = desiredJoints(i-7);
        }
    }
    return true;
}

bool InverseKinematicsIPOPT::eval_f(Ipopt::Index n, const Number* x, bool new_x, Number& obj_value)
{
    Eigen::Map< const Eigen::VectorXd > x_in (x, totalDOF);
    Eigen::Map < Eigen::VectorXd > gradientIn (gradient.data(), totalDOF);
    Eigen::VectorXd X = x_in;
    
    obj_value = 0.5*x_in.transpose()*toEigen(hessian)*x_in;
    double grad = gradientIn.transpose() * x_in;
    
    obj_value += grad;

    return true;
}

bool InverseKinematicsIPOPT::eval_grad_f(Ipopt::Index n, const Number* x, bool new_x, Number* grad_f)
{
    Eigen::Map< const Eigen::VectorXd > x_in (x, totalDOF);
    Eigen::Map< Eigen::VectorXd > Grad_f (grad_f, totalDOF);
    Grad_f.setZero();
    
    Grad_f = toEigen(hessian)*x_in + toEigen(gradient);
    
    return true;
}

bool InverseKinematicsIPOPT::eval_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Number* g)
{
    Transform p_H_e;
    Eigen::Map< const Eigen::VectorXd > x_in(x, totalDOF);
    Eigen::Map< Eigen::VectorXd > g_in(g, 8);
    VectorDynSize joints(totalDOF-7);
    
    toEigen(joints) = x_in.tail(totalDOF-7);
    iKDC.setJointPos(joints);
    p_H_e = iKDC.getRelativeTransform(parentFrame,endEffectorFrame);
    
    g_in.head<3>() = toEigen(p_H_e.getPosition()) - x_in.head<3>();
    
    g_in.segment<4>(3) = toEigen(p_H_e.getRotation().asQuaternion()) - x_in.segment<4>(3);
    
    g_in(7) = x_in.segment<4>(3).squaredNorm();
    
    return true;

}

bool InverseKinematicsIPOPT::eval_jac_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Number* values)
{
    
    if(values == NULL){
        int val = 0;
        int i = 0;
        int j = 0;
    
        //sparsity of the jacobian
        for(val = 0; val < 7; val++){
            iRow[val] = val; jCol[val] = val; //the top left corner of the matrix is diagonal
        }
        
        for(i = 0; i < 7; i++){
            for(j = 7; j < (totalDOF); j++){

                iRow[val] = i; jCol[val] = j;  //the right top part is dense
                val++;
            }
        }
        
        for(j = 3; j < 7; j++){

            iRow[val] = 7; jCol[val] = j;  //i is constantly indicating the eight column, here there are just 4 element corresponding to the elements of the quaternion in the unknown vector
            val++;
        }
    }
    else{
        Eigen::Map< const Eigen::VectorXd > x_in(x, totalDOF);
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> denseJac(m,totalDOF);
        Vector4 quaternion;
        VectorDynSize configuration;
        MatrixFixSize<7,6> map;
        MatrixDynSize jacobian;
        
        toEigen(quaternion) = x_in.segment<4>(3);
        configuration.resize(totalDOF-7);
        toEigen(configuration) = x_in.tail(totalDOF-7);
        
        twistToQuaternionTwist(quaternion, map);
        relativeJacobian(configuration,jacobian);
        
        denseJac.setZero();
        denseJac.block<7,7>(0,0) = -Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> (7,7).setIdentity();
        denseJac.topRightCorner(7,totalDOF-7) = toEigen(map) * toEigen(jacobian);
        denseJac.block<1,4>(7,3) = 2*toEigen(quaternion);
        
        int val = 0;
        int i = 0;
        int j = 0;
    
        //sparsity of the jacobian
        for(val = 0; val < 7; val++){
            values[val] = denseJac(val,val); 
        }
        
        for(i = 0; i < 7; i++){
            for(j = 7; j < (totalDOF); j++){
                values[val] = denseJac(i,j);
                val++;
            }
        }
        
        for(j = 3; j < 7; j++){

            values[val] = denseJac(7,j);
            val++;
        }
    }
    
    return true;
}

void InverseKinematicsIPOPT::finalize_solution(SolverReturn status, Ipopt::Index n, const Number* x, const Number* z_L, const Number* z_U, Ipopt::Index m, const Number* g, const Number* lambda, Number obj_value, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
{
    if((status == Ipopt::SUCCESS)||status == Ipopt::STOP_AT_ACCEPTABLE_POINT){
        VectorDynSize x_in(x, totalDOF);
        Rotation endEffectorOrientation, actualEndEffectorRotation;
        Vector4 actualEndEffectorQuaternion;
        
        toEigen(jointResult) = toEigen(x_in).tail(totalDOF-7);
        
        toEigen(positionError) = toEigen(desiredPosition) - toEigen(x_in).head(3);
        
        endEffectorOrientation.fromQuaternion(desiredQuaternion);
        toEigen(actualEndEffectorQuaternion) = toEigen(x_in).segment<4>(3);
        actualEndEffectorRotation.fromQuaternion(actualEndEffectorQuaternion);
        rotationError = actualEndEffectorRotation*(endEffectorOrientation.inverse());
        
        angleError = acos(abs(toEigen(actualEndEffectorQuaternion).dot(toEigen(desiredQuaternion).transpose()))); //Metrics for 3D rotation by Du Q.Huynh
        exitCode = 0;
    }
    else {
        switch(status){
            case(Ipopt::MAXITER_EXCEEDED):
                exitCode = -1;
                break;
                
            case(Ipopt::CPUTIME_EXCEEDED):
                exitCode = -2;
                break;
                
            case(Ipopt::STOP_AT_TINY_STEP):
                exitCode = -3;
                break;
                
            case(Ipopt::LOCAL_INFEASIBILITY):
                exitCode = -4;
                break;
                
            case(Ipopt::USER_REQUESTED_STOP):
                exitCode = -5;
                break;
                
            case(Ipopt::DIVERGING_ITERATES):
                exitCode = -6;
                break;
                
            case(Ipopt::RESTORATION_FAILURE):
                exitCode = -7;
                break;
                
            case(Ipopt::ERROR_IN_STEP_COMPUTATION):
                exitCode = -8;
                break;
                
            case(Ipopt::INVALID_NUMBER_DETECTED):
                exitCode = -9;
                break;
                
            case(Ipopt::INTERNAL_ERROR):
                exitCode = -10;
                break;
        }
    }

}

bool InverseKinematicsIPOPT::eval_h(Ipopt::Index n, const Number* x, bool new_x, Number obj_factor, Ipopt::Index m, const Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Number* values)
{
    //return Ipopt::TNLP::eval_h(n, x, new_x, obj_factor, m, lambda, new_lambda, nele_hess, iRow, jCol, values);
    return false;
}


int main(){}
