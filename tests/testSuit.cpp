#include "InverseKinematics.h"
#include "URDFdir.h"
#include "iDynTree/Model/Traversal.h"
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <matio.h>
#include <ctime>

int main(int argc, char **argv) {
    
    iDynTree::ModelLoader modelLoader;
    iDynTree::Model model;
    
    std::cerr<<"Load model from "<<getAbsModelPath("XSensURDF_subj1.urdf")<< std::endl;
    bool ok=modelLoader.loadModelFromFile(getAbsModelPath("XSensURDF_subj1.urdf"));
    model = modelLoader.model();
    
    iDynTree::assertTrue(ok);
    
    std::cerr <<"Model Loaded"<< std::endl;
    
    mat_t *pHumanState;

    std::cerr<<"Load human_state "<<getAbsModelPath("human_state.mat")<< std::endl;
    
    pHumanState = Mat_Open(getAbsModelPath("human_state.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pHumanState!=NULL);
    
    //LOADING OF human_state.mat
    
    matvar_t *humanStateVar;
    humanStateVar = Mat_VarRead(pHumanState,"human_state");
    iDynTree::assertTrue(humanStateVar != NULL);
    std::cerr<<"Found human_state variable"<<std::endl;
    
    matvar_t *humanStateQ;
    
    humanStateQ = Mat_VarGetStructFieldByName(humanStateVar, "q", 0);
    iDynTree::assertTrue(humanStateQ != NULL);
    
    std::cerr << "Dimensions: "<< humanStateQ->dims[0] << "x" << humanStateQ->dims[1] << std::endl;
    
    Eigen::Map< Eigen::MatrixXd > mapHumanState((double*)humanStateQ->data, humanStateQ->dims[0], humanStateQ->dims[1]);
    
    iDynTree::MatrixDynSize humanStateQi(humanStateQ->dims[0], humanStateQ->dims[1]);
    
    iDynTree::toEigen(humanStateQi) = mapHumanState;
    
    
    //LOAD OF selectedjoints.mat
    mat_t *pSelectedJoints;

    std::cerr<<"Load selectedjoints at "<<getAbsModelPath("selectedJoints.mat")<< std::endl;
    
    pSelectedJoints = Mat_Open(getAbsModelPath("selectedJoints.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pSelectedJoints!=NULL);
    std::cerr << "selectedjoints loaded" << std::endl;
    
    matvar_t *selectedJointsVar;
    selectedJointsVar = Mat_VarRead(pSelectedJoints,"selectedJoints");
    iDynTree::assertTrue(selectedJointsVar != NULL);
    std::cerr<<"Found 'selectedJoints' variable with dimension "<< selectedJointsVar->dims[0] << std::endl;
    
    matvar_t *cell;
    std::vector< std::string > selectedJointsVector;
    selectedJointsVector.resize(selectedJointsVar->dims[0]);
    
    for(int i=0; i<selectedJointsVar->dims[0]; ++i){
        
        cell = Mat_VarGetCell(selectedJointsVar, i);
        selectedJointsVector[i] = (char*)cell->data;
        
    }

    //LOAD OF suit.mat
    mat_t *pSuit;
    
    std::cerr<<"Load suit.mat at "<<getAbsModelPath("suit.mat")<< std::endl;
    pSuit = Mat_Open(getAbsModelPath("suit.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pSuit!=NULL);
    std::cerr << "suit loaded" << std::endl;
    
    matvar_t *suitVar;
    suitVar = Mat_VarRead(pSuit,"suit");
    iDynTree::assertTrue(humanStateVar != NULL);
    std::cerr<<"Found suit variable"<<std::endl;
    
    matvar_t *linksVar;
    linksVar = Mat_VarGetStructFieldByName(suitVar,"links",0);
    iDynTree::assertTrue(linksVar != NULL);
    std::cerr<<"Found 'links' variable with dimension "<< linksVar->dims[0] << std::endl;
    
    matvar_t *linkCell;
    
    std::vector< std::string > linksName;
    linksName.resize(linksVar->dims[0]);
    
    std::vector< iDynTree::MatrixDynSize > linksPositions;
    linksPositions.resize(linksVar->dims[0]);
    
    std::vector< iDynTree::MatrixDynSize > linksQuaternions;
    linksQuaternions.resize(linksVar->dims[0]);
    
    matvar_t *temp;
    matvar_t *tempMeas;
    
    for(int i=0; i < linksVar->dims[0] ;++i){
        
        linkCell = Mat_VarGetCell(linksVar, i);
        
        //Get Name
        temp = Mat_VarGetStructFieldByName(linkCell,"label", 0);
        iDynTree::assertTrue(temp != NULL);
        linksName[i] =(char*) temp->data;
        
        //Get meas
        tempMeas = Mat_VarGetStructFieldByName(linkCell,"meas", 0);
        iDynTree::assertTrue(tempMeas != NULL);
        
        //Get positions
        temp = Mat_VarGetStructFieldByName(tempMeas,"position", 0);
        iDynTree::assertTrue(temp != NULL);
        Eigen::Map< Eigen::MatrixXd > tempMapPosition((double*)temp->data, temp->dims[0], temp->dims[1]);
        linksPositions[i].resize(temp->dims[0], temp->dims[1]);
        toEigen(linksPositions[i]) = tempMapPosition;
        
        //Get orientations
        temp = Mat_VarGetStructFieldByName(tempMeas,"orientation", 0);
        iDynTree::assertTrue(temp != NULL);
        Eigen::Map< Eigen::MatrixXd > tempMapOrientation((double*)temp->data, temp->dims[0], temp->dims[1]);
        linksQuaternions[i].resize(temp->dims[0], temp->dims[1]);
        toEigen(linksQuaternions[i]) = tempMapOrientation;
    }
    
    //Deleting stuff
    std::cerr<< "Deleting human_state variable" << std::endl;
    Mat_VarFree(humanStateVar);
    
    std::cerr<<"Closing human_state.mat file."<<std::endl;
    Mat_Close(pHumanState);
    
        std::cerr<< "Deleting human_state variable" << std::endl;
    Mat_VarFree(selectedJointsVar);
    
    std::cerr<<"Closing selectedjoints.mat file."<<std::endl;
    Mat_Close(pSelectedJoints);
    
    std::cerr<<"Closing suit.mat file."<<std::endl;
    Mat_Close(pSuit);
    
    
    //START Test
    
    
    std::vector< InverseKinematics > solvers(22,InverseKinematics("ma57"));
    std::string parentFrame, targetFrame;
    iDynTree::Traversal traversal;
    
    
    
    std::vector < std::string > selectedJointsList = selectedJointsVector;
    std::vector< std::string >::const_iterator pickJ = selectedJointsList.begin();
    
    
    
    std::cerr << "Compute the traversal from " << model.getFrameName(0) << std::endl;
    model.computeFullTreeTraversal(traversal, 0);
    for(int i=0; i<22; ++i){
        parentFrame = linksName[i];
        targetFrame = linksName[i+1];
        solvers[i].setModel(model, parentFrame, targetFrame);
    }
}
