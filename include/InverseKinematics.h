#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "InverseKinematicsV2IPOPT.h"
#include <string>

class InverseKinematics {
    Ipopt::SmartPtr< InverseKinematicsV2IPOPT > solverPointer;
    Ipopt::SmartPtr< Ipopt::IpoptApplication > loader;
    bool updated;
    bool initialized;
    bool alreadyOptimized;
    std::string _parentFrame;
    std::string _targetFrame;

    bool update();
    bool getReducedModel(const iDynTree::Model& fullModel, const std::vector< std::string >& consideredJoints, iDynTree::Model& modelOutput);
    bool getFrameIndeces(const std::string& parentFrame, const std::string& endEffectorFrame, const iDynTree::Model model, iDynTree::FrameIndex& parentFrameIndex, iDynTree::FrameIndex& endEffectorFrameIndex);
    void removeUnsupportedJoints(const iDynTree::Model& modelInput, iDynTree::Model& modelOutput);
    bool autoSelectJointsFromTraversal(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, iDynTree::Model& modelOutput);
    
public:
    
    /**
     * \brief Constructor
     * It creates an InverseKinematicsV2IPOPT object together with an istance of IpoptApplication.
     * \note Don't use copies of this object, otherwise the two copy will point to the same solver object.
     */
    InverseKinematics();
    
    /**
     * \brief Constructor
     * It creates an InverseKinematicsV2IPOPT object together with an istance of IpoptApplication, while selecting solverName as a solver. 
     * \param[in] solverName The name of the solver.
     * \warning The solver should be installed within Ipopt, or the file libhsl.so should be available, pointing the right solver. An error will be thrown otherwise.
     * \note Don't use copies of this object, otherwise the two copy will point to the same solver object.
     * 
     */
    InverseKinematics(const std::string& solverName);
    
    /**
     * Destructor
     */
    ~InverseKinematics();

    /**
     * \brief Allows to define the file from which to load the model, and the frame to be used as reference and target when computing inverse kinematics.
     * \param[in] URDFfilename It is the adress of the .xml file cointaining the description of the robot model.
     * \param[in] parentFrame Name of the frame to be considered as a reference when computing inverse kinematics.
     * \param[in] endEffectorFrame Name of the frame to be used as a target when computing inverse kinematics.
     * \param[in] consideredJoints List of joints to be considered when computing inverse kinematics. In the case the first element of this vector cointains "All" (or "ALL" or "all"), 
     * all the joints described in the URDF are considered.
     * \note If consideredJoints is empty, the joints between parentFrame and endEffectorFrame will be automatically selected.
     * \warning Only 1DOF joints are supported. All the other joints will be automatically ignored. A warning message will be throwed in this case.
     * \return It returns true in the case the model has been successfully loaded. An error messaged is throwed otherwise.
     */
    bool setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame, const std::vector< std::string >& consideredJoints);
    
    /**
     * \brief Allows to define the file from which to load the model, and the frame to be used as reference and target when computing inverse kinematics. 
     * It automatically selects all the joints between
     * parentFrame and endEffectorFrame.
     * \param[in] URDFfilename It is the adress of the .xml file cointaining the description of the robot model.
     * \param[in] parentFrame Name of the frame to be considered as a reference when computing inverse kinematics.
     * \param[in] endEffectorFrame Name of the frame to be used as a target when computing inverse kinematics.
     * \warning Only 1DOF joints are supported. All the other joints will be automatically ignored. A warning message will be throwed in this case.
     * \return It returns true in the case the model has been successfully loaded. An error messaged is throwed otherwise.
     */
    bool setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame);
    
    /**
     * \brief Allows to define directly the model the frames to be used when computing inverse kinematics.
     * \param[in] modelInput The model to be loaded.
     * \param[in] parentFrame Name of the frame to be considered as a reference when computing inverse kinematics.
     * \param[in] endEffectorFrame Name of the frame to be used as a target when computing inverse kinematics.
     * \param[in] autoSelectJoints (Default TRUE) If true, it lets the solver to automatically select the joints between parentFrame and endEffectorFrame from those already available in the model.
     * \warning Only 1DOF joints are supported. All the other joints will be automatically ignored. A warning message will be throwed in this case.
     * \return It returns true in the case the model has been successfully loaded. An error messaged is throwed otherwise.
     */
    bool setModel(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, const bool autoSelectJoints = true);
    
    /**
     * \brief (Optional) Set the desired weights to be used during the optimization process.
     * \param[in] weights It is a 3D vector which weights in a different way the square norm on the position error (first element), the square norm of the orientation error expressed in quaternion 
     * (second element), and the square norm between the actual and the desired joints'configuration.
     * \note The default is [100, 100, 0.01];
     */
    void setWeights(const iDynTree::Vector3& weights);
    
    /**
     * \brief Defines the desired position for the endEffectorFrame origin.
     * \param[in] desiredPosition Desired 3D position for the origin of endEffectorFrame, expressed in the parentFrame frame of reference.
     */
    void setDesiredPosition(const iDynTree::Position& desiredPosition);
    
    /**
     * \brief Defines the desired quaternion.
     * It expresses the orientation the endEffectorFrame should have with respect to the parentFrame.
     * \param[in] desiredQuaternion It defines the orientation the endEffectorFrame should have, expressed in the parentFrame frame of reference;
     */
    void setDesiredQuaternion(const iDynTree::Vector4& desiredQuaternion); 
    
    /**
     * \brief Defines the relative transformation the endEffectorFrame should have with respect to the parentFrame.
     * \param[in] p_H_t The position and orientation the endEffectorFrame should have with respect to the parentFrame, based on which the inverse kinematics should be computed.
     */
    void setDesiredTransformation(const iDynTree::Transform& p_H_t);
    
    void setDesiredJointPositions(const iDynTree::VectorDynSize& desiredJoints);
    
    void setGuess(const iDynTree::VectorDynSize& guess);
    
    bool setRandomGuess(const double feed, iDynTree::VectorDynSize& guess);
    
    bool update(const iDynTree::Vector3& gains, const iDynTree::Position &desiredPosition, const iDynTree::Vector4 &desiredQuaternion, const iDynTree::VectorDynSize &desiredJoints);
    
    signed int runIK(iDynTree::VectorDynSize& jointsOut);

    bool getErrors(iDynTree::Position& positionError, iDynTree::Rotation& rotationError, double* angleError);
    
    void getFrames(std::string& parentFrame, std::string& endEffectorFrame);
    
    bool getConsideredJoints(std::vector< std::string >& consideredJoints);
};
#endif /* end of include guard: INVERSEKINEMATICS_H */
