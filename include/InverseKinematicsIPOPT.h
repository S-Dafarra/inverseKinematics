#ifndef INVERSEKINEMATICSIPOPT_H
#define INVERSEKINEMATICSIPOPT_H

#include <IpTNLP.hpp>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <math.h>
#include <IpIpoptApplication.hpp>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>

class InverseKinematicsIPOPT : public Ipopt::TNLP {
    iDynTree::Model model;
    bool modelLoaded;
    bool gainsLoaded;
    bool framesLoaded;
    iDynTree::KinDynComputations iKDC;
    iDynTree::MatrixDynSize hessian;
    iDynTree::VectorDynSize gradient;
    iDynTree::FrameIndex parentFrame, endEffectorFrame;
    std::vector<double> gains;
    iDynTree::Vector3 desiredPosition;
    iDynTree::Vector4 desiredQuaternion; 
    iDynTree::VectorDynSize desiredJoints;
    std::vector< std::pair<double,double> > jointsLimits; //joints upper and lower limits 
    int totalDOF;
    iDynTree::MatrixDynSize Ep, Eq, Edof; //extractors from the total variable vector    
    iDynTree::VectorDynSize jointResult;
    iDynTree::Vector3 positionError;
    iDynTree::Rotation rotationError;
    double angleError;
    signed int exitCode;
    
public:
    InverseKinematicsIPOPT();
    
    InverseKinematicsIPOPT(const std::string &filename, const std::vector< std::string > &consideredJoints, const std::vector<double>& gainsIn, const iDynTree::Vector3 &desiredPositionIn, const iDynTree::Vector4 &desiredQuaternionIn, const iDynTree::VectorDynSize &desiredJointsIn, const std::string& parentFrameIn, const std::string& endEffectorFrameIn);

    virtual ~InverseKinematicsIPOPT();

    virtual bool load(const std::string &filename, const std::vector< std::string > &consideredJoints);
    
    virtual bool setFrames(const std::string& parentFrameIn, const std::string& endEffectorFrameIn);

    virtual bool update(const std::vector<double>& gainsIn, const iDynTree::Vector3 &desiredPositionIn, const iDynTree::Vector4 &desiredQuaternionIn, const iDynTree::VectorDynSize &desiredJointsIn);

    virtual bool update();

    virtual void twistToQuaternionTwist(iDynTree::Vector4 &quaternion, iDynTree::MatrixFixSize<7,6>& mapOut);

    virtual void relativeJacobian(const iDynTree::VectorDynSize &configuration, iDynTree::MatrixDynSize& jacobianOut);

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda);

    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Number& obj_value);

    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                             Ipopt::Number* grad_f);

    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Index m, Ipopt::Number* g);

    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                            Ipopt::Index *jCol, Ipopt::Number* values);

    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values);

    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                   const Ipopt::Number* x, const Ipopt::Number* z_L,
                                   const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                   const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq);
};

#endif 