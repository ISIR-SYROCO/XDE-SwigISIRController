// isir_task_manager.i - SWIG interface to use ISIRTaskManager

%module isir_task_manager
%{

#define SWIG_FILE_WITH_INIT

#include <ISIRTaskManager/mathutils.hpp>
#include <ISIRTaskManager/taskxmlparser.hpp>
#include <ISIRTaskManager/trajectory.hpp>

%}

%include "std_string.i"

%ignore TaskXMLParser::parse;
%ignore TaskXMLParser::addTask;
%ignore TaskXMLParser::updateTask;

%ignore TaskXMLParser::parseTaskInfo;

%ignore TaskXMLParser::fillVector;
%ignore TaskXMLParser::fillVector3;

%ignore TaskXMLParser::parseFeatureFullState;
%ignore TaskXMLParser::parseObjectiveFullState;

%ignore TaskXMLParser::parseFeaturePartialState;
%ignore TaskXMLParser::parseObjectivePartialState;

%ignore TaskXMLParser::parseObjectiveDisplacementFrame;
%ignore TaskXMLParser::parseObjectiveOrientationFrame;
%ignore TaskXMLParser::parseObjectivePositionFrame;
%ignore TaskXMLParser::parseObjectiveCoM;

%ignore TaskXMLParser::parseFeatureDisplacement;

%ignore TaskXMLParser::parseFeatureOrientation;

%ignore TaskXMLParser::parseFeaturePosition;
%ignore TaskXMLParser::parseFeatureCom;
%ignore TaskXMLParser::parseFeatureContact;

%ignore TaskXMLParser::parseParam;

%ignore TaskXMLParser::parseFrameTaskDofs;

%ignore TaskXMLParser::parseLocalOffset;

%ignore TaskXMLParser::initTask;

%ignore TaskXMLParser::printFullstateDesc;
%ignore TaskXMLParser::printPartialstateDesc;
%ignore TaskXMLParser::printDisplacementDesc;

//%include "ISIRTaskManager/mathutils.hpp"
%include "ISIRTaskManager/taskxmlparser.hpp"
%include "ISIRTaskManager/trajectory.hpp"

//Cast function
%inline %{
    fullstate_task_t& fullstateCast(task_t& task){
        return *dynamic_cast<fullstate_task_t*>(&task);
    }

    partialstate_task_t& partialstateCast(task_t& task){
        return *dynamic_cast<partialstate_task_t*>(&task);
    }

    frame_task_t& frameCast(task_t& task){
        return *dynamic_cast<frame_task_t*>(&task);
    }

    displacement_task_t& displacementCast(task_t& task){
        return *dynamic_cast<displacement_task_t*>(&task);
    }

    position_task_t& positionCast(task_t& task){
        return *dynamic_cast<position_task_t*>(&task);
    }

    orientation_task_t& orientationCast(task_t& task){
        return *dynamic_cast<orientation_task_t*>(&task);
    }

    com_task_t& comCast(task_t& task){
        return *dynamic_cast<com_task_t*>(&task);
    }

    contact_task_t& contactCast(task_t& task){
        return *dynamic_cast<contact_task_t*>(&task);
    }
    
    TrajectoryReaderFullJoint& trajectoryReaderFullJointCast(Trajectory* traj){
        return *dynamic_cast<TrajectoryReaderFullJoint*>(traj);
    }
%}
