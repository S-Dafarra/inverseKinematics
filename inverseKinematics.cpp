#include "inverseKinematics.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <cassert>

using namespace std;
using namespace iDynTree;
using namespace Ipopt;


bool inverseKinematics::load(const string& filename)
{
    bool success= false;
    ModelLoader loader;

    success = loader.loadModelFromFile(filename);
    if (success)
        success =  loader.isValid();

    model = loader.model();

    return success;
}

bool inverseKinematics::configure(const string& filename, const vector< string >& consideredJoints, const vector< double >& gains, const Vector3& desiredPosition, const Vector4& desiredQuaternion, const VectorDynSize& desiredJoints, const string& parentFrameIn, const string& endEfFrameIn)
{
    bool success= false;
    ModelLoader loader;
    
    success = inverseKinematics::load(filename);
    
    const Model input=model;
    
    if (success && !consideredJoints.empty()){
        success = loader.loadReducedModelFromFullModel(input, consideredJoints);
        success = success && loader.isValid();
    }
    
    if (!success)
        return false;
    
    model = loader.model();
    
    iKDC.loadRobotModel(model);
    iKDC.setJointPos(desiredJoints); //set the initial position equal to desired joints
    
    jointsLimits.clear();
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
    
    totalDOF =7 + model.getNrOfDOFs();
    
    parentFrame = model.getFrameIndex(parentFrameIn);
    endEfFrame = model.getFrameIndex(endEfFrameIn);
    
    assert((parentFrame != FRAME_INVALID_INDEX)&&(endEfFrame != FRAME_INVALID_INDEX));
    
    Hessian.resize(totalDOF,totalDOF);
    Gradient.resize(totalDOF);
    
    MatrixDynSize Ep, Eq, Edof; //extractors from the total variable vector
    
    Ep.resize(3,totalDOF);
    Ep.zero();
    toEigen(Ep).block<3,3>(0,0).setIdentity();
    
    Eq.resize(4,totalDOF);
    Eq.zero();
    toEigen(Eq).block<4,4>(0,3).setIdentity();
    
    Edof.resize(model.getNrOfDOFs(),totalDOF);
    toEigen(Edof).block(0,7,model.getNrOfDOFs(),model.getNrOfDOFs()).setIdentity();
    
    assert(gains.size() > 2);
    assert(desiredJoints.size() == model.getNrOfDOFs());
    
    toEigen(Hessian) = (gains[0]*toEigen(Ep).transpose()*toEigen(Ep) + gains[1]*toEigen(Eq).transpose()*toEigen(Eq) + gains[2]*toEigen(Edof).transpose()*toEigen(Edof));
    toEigen(Gradient) = -(gains[0]*toEigen(desiredPosition).transpose()*toEigen(Ep) + gains[1]*toEigen(desiredQuaternion).transpose()*toEigen(Eq) + gains[2]*toEigen(desiredJoints).transpose()*toEigen(Edof));
    
    return true;
}

MatrixFixSize<7,6> inverseKinematics::twistToQuatTwist(iDynTree::Vector4 &quat)
{
    MatrixFixSize<7,6> map;
    MatrixFixSize<3,4> omegaToQuat;
    Eigen::Vector3d imQuat;
    IndexRange rowRange, columnRange;
    
    imQuat = toEigen(quat).tail(3);
    
    omegaToQuat.zero();
    
    toEigen(omegaToQuat).leftCols<1>() = -imQuat;
    toEigen(omegaToQuat).block<3,3>(0,1).setIdentity();
    toEigen(omegaToQuat).block<3,3>(0,1) *= quat(0);
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

    return map;
}

MatrixDynSize inverseKinematics::relativeJacobian(const VectorDynSize& configuration)
{
    Matrix6x6 right2mixed, left2mixed;
    MatrixDynSize e_J_we_temp, p_J_wp_temp, e_J_we, p_J_wp, JacobianOut;
    
    iKDC.setJointPos(configuration);
    iKDC.setFrameVelocityRepresentation(BODY_FIXED_REPRESENTATION); //left trivialized velocity
    
    left2mixed = iKDC.getRelativeTransformExplicit(endEfFrame,parentFrame,endEfFrame,endEfFrame).asAdjointTransform(); //is the adjoint transformation from left-trivialized velocity to mixed velocity with the origin on the target frame and the orientation of the parent frame
    right2mixed = iKDC.getRelativeTransformExplicit(endEfFrame,parentFrame,parentFrame,parentFrame).asAdjointTransform(); //is the adjoit trasnformation from right-trivialized velocity to mixed velocity. It comes from a multiplication of two adjoints: from left to mixed times from right to left
    
    iKDC.getFrameFreeFloatingJacobian(parentFrame, p_J_wp_temp); //getting the jacobian from world to parent frame with left-trivialized velocity representation
    iKDC.getFrameFreeFloatingJacobian(endEfFrame, e_J_we_temp);  //getting the jacobian from world to target frame with left-trivialized velocity representation
    
    e_J_we.resize(6,totalDOF-7);
    p_J_wp.resize(6,totalDOF-7);
    
    toEigen(e_J_we) = toEigen(e_J_we_temp).rightCols(totalDOF-7); //removing the base contribution
    toEigen(p_J_wp) = toEigen(p_J_wp_temp).rightCols(totalDOF-7);
    
    JacobianOut.resize(6,totalDOF-7);
    toEigen(JacobianOut) = toEigen(left2mixed)*toEigen(e_J_we) - toEigen(right2mixed)*toEigen(p_J_wp);
    
    return JacobianOut;
}


bool inverseKinematics::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = totalDOF; //position and orientation of the end effector (3 for position and 4 for the quaternion of the orientation) + joints'DoF, in this order.
    m = 8; //7 for forward kinematics (relates the 7 dofs of the end effector with the joint dofs) + 1 on the norm of the quaternion.
    
    nnz_jac_g = 7 + (totalDOF-7)*7 + 4;
    nnz_h_lag = n*n; //dense
    
    index_style = Ipopt::TNLP::C_STYLE;
    
    return true;
}

bool inverseKinematics::get_bounds_info(Ipopt::Index n, Number* x_l, Number* x_u, Ipopt::Index m, Number* g_l, Number* g_u)
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

bool inverseKinematics::get_starting_point(Ipopt::Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Ipopt::Index m, bool init_lambda, Number* lambda)
{
    Transform p_H_e; //from parent to end effector
    Eigen::Map< Eigen::VectorXd > x_e (x, 7);
    VectorDynSize jointDes;
 
    if(init_x){
        p_H_e = iKDC.getRelativeTransform(parentFrame,endEfFrame);
        x_e.head<3>() = toEigen(p_H_e.getPosition());
        x_e.tail<4>() = toEigen(p_H_e.getRotation().asQuaternion());
        
        iKDC.getJointPos(jointDes);
        for(int i = 7; i < totalDOF; i++){
            x[i] = jointDes(i-7);
        }
    }
    
    if(init_z) return false;
    if(init_lambda) return false;
}

bool inverseKinematics::eval_f(Ipopt::Index n, const Number* x, bool new_x, Number& obj_value)
{
    Eigen::Map< const Eigen::VectorXd > x_in (x, totalDOF);
    
    obj_value = 0.5*x_in.transpose()*toEigen(Hessian)*x_in + (toEigen(Gradient)*x_in)[0];

    return true;
}

bool inverseKinematics::eval_grad_f(Ipopt::Index n, const Number* x, bool new_x, Number* grad_f)
{
    Eigen::Map< const Eigen::VectorXd > x_in (x, totalDOF);
    Eigen::Map< Eigen::VectorXd > Grad_f (grad_f, totalDOF);
    
    Grad_f = toEigen(Hessian)*x_in + toEigen(Gradient);
    
    return true;
}

bool inverseKinematics::eval_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Number* g)
{
    Transform p_H_e;
    Eigen::Map< const Eigen::VectorXd > x_in(x, totalDOF);
    Eigen::Map< Eigen::VectorXd > g_in(g, 8);
    VectorDynSize joints(totalDOF);
    
    toEigen(joints) = x_in.tail(totalDOF-7);
    iKDC.setJointPos(joints);
    p_H_e = iKDC.getRelativeTransform(parentFrame,endEfFrame);
    
    g_in.head<3>() = toEigen(p_H_e.getPosition()) - x_in.head<3>();
    
    g_in.segment<4>(3) = toEigen(p_H_e.getRotation().asQuaternion()) - x_in.segment<4>(3);
    
    g_in(7) = x_in.segment<4>(3).squaredNorm();
    
    return true;

}

bool inverseKinematics::eval_jac_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Number* values)
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
                val++;
                iRow[val] = i; jCol[val] = j;  //the right top part is dense
            }
        }
        
        for(j = 3; j < 7; j++){
            val++;
            iRow[val] = 7; jCol[val] = j;  //i is constantly indicating the eight column, here there are just 4 element corresponding to the elements of the quaternion in the unknown vector
        }
    }
    else{
        Eigen::Map< const Eigen::VectorXd > x_in(x, totalDOF);
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> denseJac(m,totalDOF);
        Vector4 quaternion;
        VectorDynSize configuration;
        
        toEigen(quaternion) = x_in.segment<4>(3);
        configuration.resize(totalDOF-7);
        toEigen(configuration) = x_in.tail(totalDOF-7);
        
        denseJac.setZero();
        denseJac.block<7,7>(0,0) = -Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> (7,7).setIdentity();
        denseJac.topRightCorner(7,totalDOF-7) = toEigen(twistToQuatTwist(quaternion)) * toEigen(relativeJacobian(configuration));
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
                val++;
                values[val] = denseJac(i,j);  
            }
        }
        
        for(j = 3; j < 7; j++){
            val++;
            values[val] = denseJac(7,j);
        }
    }
    
    return true;
}


int main(){}