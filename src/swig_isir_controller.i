// swig_isir_controller.i - SWIG interface to get controller interface from orcisir_ISIRController

%module swig_isir_controller
%{
#define SWIG_FILE_WITH_INIT

#include "orcisir/Model.h"

#include "orcisir/Solvers/OneLevelSolver.h"

#include "orc/control/FullState.h"
#include "orcisir/Features/ISIRPartialState.h"
#include "orcisir/Features/ISIRFeature.h"

#include "orcisir/ISIRController.h"

#include "orcisir/Constraints/TorqueLimitConstraint.h"
#include "orcisir/Constraints/JointLimitConstraint.h"
#include "orcisir/Constraints/ContactAvoidanceConstraint.h"
%}

%feature("autodoc", "1");
%include "interfaces/isir_controller_docstrings.i"

%include "std_string.i"

%include "interfaces/typemap_eigen_lgsm.i"


//Management of exceptions raised by XDE-SwigISIRController
%include exception.i
%exception { 
    try {
        $action
    } catch(std::runtime_error &e) {
        std::string err_msg("std::runtime_error exception raised by XDE-SwigISIRController:\n");
        err_msg += e.what();
        err_msg += "\n";
        SWIG_exception(SWIG_RuntimeError, err_msg.c_str());
    } catch (...) {
        SWIG_exception(SWIG_RuntimeError, "unknown exception raised by XDE-wigISIRController!");
    }
}



////////////////////////////////////////////////////////////////////////////////
//
// orc::NamedInstance base class interface
//
////////////////////////////////////////////////////////////////////////////////
%include  "orc/optim/NamedInstance.h"    // for base class interface


////////////////////////////////////////////////////////////////////////////////
//
// orc::Model base class interface
//
////////////////////////////////////////////////////////////////////////////////
//IN orc::Model ==> ignore all method that are not useful in python interface
%ignore orc::Model::getConfigurationVariable;
%ignore orc::Model::getVelocityVariable;
%ignore orc::Model::getAccelerationVariable;
%ignore orc::Model::getJointTorqueVariable;
%ignore orc::Model::getRootConfigurationVariable;
%ignore orc::Model::getInternalConfigurationVariable;
%ignore orc::Model::getRootVelocityVariable;
%ignore orc::Model::getInternalVelocityVariable;
%ignore orc::Model::getRootAccelerationVariable;
%ignore orc::Model::getInternalAccelerationVariable;
%ignore orc::Model::getModelContacts; //TODO: yes or no??
%include  "orc/control/Model.h"

%include "orcisir/Model.h"


////////////////////////////////////////////////////////////////////////////////
//
// orcisir::Solver base class interface
//
////////////////////////////////////////////////////////////////////////////////
namespace Eigen {}
%ignore orc::testUtilities;
%ignore orc::utils::printSolution;
%include "orc/optim/SolverUtilities.h"
%include "orc/optim/Solver.h"

%ignore orcisir::SquaredLinearObjective;
%include "orcisir/Solvers/ISIRSolver.h"
%include "orcisir/Solvers/OneLevelSolver.h"


////////////////////////////////////////////////////////////////////////////////
//
// orcisir::Constraint base class interface
//
////////////////////////////////////////////////////////////////////////////////
%ignore orcisir::ISIRDynamicFunction;
%include "orcisir/Constraints/ISIRConstraint.h"

%ignore orcisir::TorqueLimitFunction;
%include "orcisir/Constraints/TorqueLimitConstraint.h"

%ignore orcisir::JointLimitFunction;
%ignore orcisir::FullJointLimitFunction;
%ignore orcisir::ReducedJointLimitFunction;
%include "orcisir/Constraints/JointLimitConstraint.h"

%ignore orcisir::ContactAvoidanceFunction;
%ignore orcisir::FullContactAvoidanceFunction;
%ignore orcisir::ReducedContactAvoidanceFunction;
%include "orcisir/Constraints/ContactAvoidanceConstraint.h"


////////////////////////////////////////////////////////////////////////////////
//
// orcisir::Task base class interface
//
////////////////////////////////////////////////////////////////////////////////
%include "orc/control/FullState.h"
%include "orcisir/Features/ISIRPartialState.h"

%ignore orc::computeDimensionFor;
%include "orc/control/ControlEnum.h"
%include "orc/control/ControlFrame.h"

%ignore orc::ContactConstraintFeature;
%ignore orc::LookAtFeature;
%include "orc/control/Feature.h"
%include "orcisir/Features/ISIRFeature.h"

////////////////////////////////////////////////////////////////////////////////
//
// orcisir::Task base class interface
//
////////////////////////////////////////////////////////////////////////////////
%include "orc/control/Task.h"
%include "orcisir/Tasks/ISIRTask.h"

////////////////////////////////////////////////////////////////////////////////
//
// orcisir::Controller base class interface
//
////////////////////////////////////////////////////////////////////////////////
%include "orc/control/Controller.h"

%ignore orcisir::ISIRController::writePerformanceInStream;
%include "orcisir/ISIRController.h"



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
//
// OROCOS Components
//
////////////////////////////////////////////////////////////////////////////////

#ifdef OROCOS_IS_AVAILABLE

%include "interfaces/isir_controller_orocos_components.i"

#endif



////////////////////////////////////////////////////////////////////////////////
//
// If available, define a getModelFromXDEDynamicModel to interface
// 'xde::gvm::extra::DynamicModel' in 'orc::Model'
//
////////////////////////////////////////////////////////////////////////////////
#ifdef XDECORE_IS_AVAILABLE

%include "interfaces/isir_controller_xde_model.i"

#endif







