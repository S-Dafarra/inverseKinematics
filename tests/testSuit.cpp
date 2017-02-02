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
    std::cerr<< "Deleting human_state variables" << std::endl;
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
    
    
    std::vector< InverseKinematics* > solvers(linksName.size()-1);
    
    for(int solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){
        solvers[solverIterator] = new InverseKinematics("ma57");
    }
    
    
    
    std::string parentFrame, targetFrame;
    int solverIterator = 0;
    std::vector < iDynTree::JointIndex > selectedJointsList(model.getNrOfJoints());
    
    for (int i = 0; i < model.getNrOfJoints(); ++i){ //let's create a checklist will all the joint indeces
        selectedJointsList[i] = i;
    }
    
    std::vector< iDynTree::JointIndex >::iterator pickJ = selectedJointsList.begin();
    std::vector< iDynTree::JointIndex >::iterator findJ = selectedJointsList.begin();
    iDynTree::LinkIndex tempFirst, tempSecond, childLink;


    do{
        pickJ = selectedJointsList.begin();
        tempFirst = model.getJoint( *pickJ )->getFirstAttachedLink();
        tempSecond = model.getJoint( *pickJ )->getSecondAttachedLink();
        while((std::find(linksName.begin(),linksName.end(),model.getFrameName(tempFirst)) ==  linksName.end()) &&       //first find a joint which is connected to a link contained in linksName (thus does not connect two fake links)
                (std::find(linksName.begin(),linksName.end(),model.getFrameName(tempSecond)) ==  linksName.end()) ){
            
            pickJ++;
            iDynTree::assertTrue(pickJ < selectedJointsList.end());
            
            tempFirst = model.getJoint(*pickJ)->getFirstAttachedLink();
            tempSecond = model.getJoint(*pickJ)->getSecondAttachedLink();
            
        }
        
        if(std::find(linksName.begin(),linksName.end(),model.getFrameName(tempFirst)) !=  linksName.end()){ //first link is a good one
            parentFrame = model.getFrameName(tempFirst);
            childLink = tempSecond;
            selectedJointsList.erase(pickJ); //remove joints already considered
        }
        else{ //second link is the good one
            parentFrame = model.getFrameName(tempSecond);
            childLink = tempFirst;
            selectedJointsList.erase(pickJ); //remove joints already considered
        }
        
        //now we have to test whether the child is a fake link or not
        while(std::find(linksName.begin(),linksName.end(),model.getFrameName(childLink)) ==  linksName.end()){
            findJ = selectedJointsList.begin();
            while( (model.getJoint( *findJ )->getFirstAttachedLink() != childLink) &&    //first find the other joint connected to the fake link
                (model.getJoint( *findJ )->getSecondAttachedLink() != childLink) ){
            
            ++findJ;
            iDynTree::assertTrue(findJ < selectedJointsList.end());

            }
            
            if(model.getJoint( *findJ )->getFirstAttachedLink() == childLink){ //first link is a good one
                childLink = model.getJoint( *findJ )->getSecondAttachedLink();
                selectedJointsList.erase(findJ);
            }
            else{ //second link is the good one
                childLink = model.getJoint( *findJ )->getFirstAttachedLink();
                selectedJointsList.erase(findJ);
            }
        }
        
        targetFrame = model.getFrameName(childLink);
        
        iDynTree::assertTrue(solverIterator < solvers.size());
        std::cerr << "Solver #" << solverIterator + 1 << std::endl;
        ok = solvers[solverIterator]->setModel(model, parentFrame, targetFrame);
        iDynTree::assertTrue(ok);
        solverIterator++;
        
    }while(selectedJointsList.size()!=0);

    iDynTree::assertTrue(solverIterator == solvers.size());
    
    iDynTree::Vector3 weights;
    weights(0) = 100;
    weights(1) = 10;
    weights(2) = 0.01;
    
    std::vector< std::string > tempConsideredJoints;
    iDynTree::VectorDynSize tempDesiredJoints;
    
    for(solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){ //set gains and desired joints positions
        solvers[solverIterator]->setWeights(weights);
        
        solvers[solverIterator]->getConsideredJoints(tempConsideredJoints);
        tempDesiredJoints.resize(tempConsideredJoints.size());
        tempDesiredJoints.zero();
        solvers[solverIterator]->setDesiredJointPositions(tempDesiredJoints);
    }
    
    int selectedInstant;
    iDynTree::Transform w_H_parent;
    iDynTree::Transform w_H_target;
    std::string tempParentFrame;
    std::string tempTargetFrame;
    iDynTree::Position tempPosition;
    iDynTree::Vector4 tempQuaternion;
    iDynTree::Rotation tempRotation;
    iDynTree::Transform tempTransform;
    int namePosition;
    iDynTree::VectorDynSize jointsOut;
    clock_t now;
    double elapsed_time;
    for(int nInstants = 0; nInstants < 3; ++nInstants){
        srand ( clock() );
        selectedInstant = rand() % humanStateQi.cols();  //random time instant
        std::cerr << "Selected time instant: " << selectedInstant << std::endl;
        
        for(solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){
            solvers[solverIterator]->getFrames(tempParentFrame, tempTargetFrame);
           /* std::cerr << "PreSolver #" << solverIterator << " parent:" << tempParentFrame << " target: "<< tempTargetFrame << std::endl;
            solvers[solverIterator].getConsideredJoints(tempConsideredJoints);
            std::cerr << "Considered joints are:"<< std::endl;
            for (std::vector< std::string >::const_iterator i = tempConsideredJoints.begin(); i != tempConsideredJoints.end(); ++i){
                std::cerr <<"-"<< *i << std::endl;
            }*/
            
            namePosition = std::distance(linksName.begin(), std::find(linksName.begin(),linksName.end(),tempParentFrame)); //search for its position in linksName
            
            iDynTree::toEigen(tempPosition) = iDynTree::toEigen(linksPositions[namePosition]).col(selectedInstant);
            iDynTree::toEigen(tempQuaternion) = iDynTree::toEigen(linksQuaternions[namePosition]).col(selectedInstant);
            tempRotation =  iDynTree::Rotation::RotationFromQuaternion(tempQuaternion);
            w_H_parent.setPosition(tempPosition);
            w_H_parent.setRotation(tempRotation);
            
            namePosition = std::distance(linksName.begin(), std::find(linksName.begin(),linksName.end(),tempTargetFrame)); 
            
            iDynTree::toEigen(tempPosition) = iDynTree::toEigen(linksPositions[namePosition]).col(selectedInstant);
            iDynTree::toEigen(tempQuaternion) = iDynTree::toEigen(linksQuaternions[namePosition]).col(selectedInstant);
            tempRotation =  iDynTree::Rotation::RotationFromQuaternion(tempQuaternion);
            
            w_H_target.setPosition(tempPosition);
            w_H_target.setRotation(tempRotation);
            tempTransform = w_H_parent.inverse()*w_H_target;
            solvers[solverIterator]->setDesiredTransformation(tempTransform);
            
            std::cerr << "Solver #" << solverIterator+1 << std::endl;
            now = clock();
            solvers[solverIterator]->runIK(jointsOut);
            elapsed_time = clock() - now;
            elapsed_time = elapsed_time/CLOCKS_PER_SEC;
            
            solvers[solverIterator]->getConsideredJoints(tempConsideredJoints);
            
            for(int jIterator = 0; jIterator < tempConsideredJoints.size(); ++jIterator){
                namePosition = std::distance(selectedJointsVector.begin(), std::find(selectedJointsVector.begin(),selectedJointsVector.end(), tempConsideredJoints[jIterator])); 
                std::cerr << tempConsideredJoints[jIterator] <<": (IK) " << jointsOut(jIterator) << " vs " << humanStateQi(namePosition,selectedInstant) << " (OpenSim)" << std::endl;
            }
            std::cerr << "Elapsed time: "<< elapsed_time<<std::endl;
            
        }
        
    }
    
    for(int solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){
        delete(solvers[solverIterator]);
    }
    
}
