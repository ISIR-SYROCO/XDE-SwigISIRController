
// File: index.xml

// File: classorcisir_1_1_contact_avoidance_constraint.xml
%feature("docstring") orcisir::ContactAvoidanceConstraint "C++
includes: ContactAvoidanceConstraint.h ";

%feature("docstring")
orcisir::ContactAvoidanceConstraint::ContactAvoidanceConstraint "ContactAvoidanceConstraint::ContactAvoidanceConstraint(const
orc::Model &model, double hpos, double margin) ";

%feature("docstring")
orcisir::ContactAvoidanceConstraint::~ContactAvoidanceConstraint "virtual
orcisir::ContactAvoidanceConstraint::~ContactAvoidanceConstraint() ";

%feature("docstring")
orcisir::ContactAvoidanceConstraint::getHorizonOfPrediction "double
ContactAvoidanceConstraint::getHorizonOfPrediction() const ";

%feature("docstring")
orcisir::ContactAvoidanceConstraint::setHorizonOfPrediction "void
ContactAvoidanceConstraint::setHorizonOfPrediction(double newHpos) ";

%feature("docstring")  orcisir::ContactAvoidanceConstraint::getMargin
"double ContactAvoidanceConstraint::getMargin() const ";

%feature("docstring")  orcisir::ContactAvoidanceConstraint::setMargin
"void ContactAvoidanceConstraint::setMargin(double newMargin) ";

%feature("docstring")
orcisir::ContactAvoidanceConstraint::updateContactInformation "void
ContactAvoidanceConstraint::updateContactInformation(const
Eigen::MatrixXd &_JObst, const Eigen::VectorXd &_dJdqOst, const
Eigen::VectorXd &_distObst, const Eigen::VectorXd &_velObst) ";


// File: classorcisir_1_1_contact_avoidance_function.xml
%feature("docstring") orcisir::ContactAvoidanceFunction "

Create a linear function that represents the joint limit function.

The contact avoidance constraint is written at first as: $ \\\\vec{0}
< \\\\vec{d}_{oa} $ where $ \\\\vec{d}_{oa} $ is the contatenation of
the minimal distances between the couples of shapes that should not
collide.

To relate this constraint with the dynamic variables, we constrain the
estimated future distances to remain positive, as explained in
orcisir::JointLimitFunction .

In this case, the constraint is expressed as follows:

\\\\begin{align*} \\\\vec{d}_{oa} - \\\\vec{m} &> \\\\vec{d}_{oa}(h)
\\\\quad \\\\text{at instant h} \\\\\\\\ \\\\vec{d}_{oa} - \\\\vec{m}
&> \\\\dot{\\\\vec{d}}_{oa} h + \\\\ddot{\\\\vec{d}}_{oa}
\\\\frac{h^2}{2} \\\\\\\\ \\\\vec{d}_{oa} - \\\\vec{m} &>
\\\\dot{\\\\vec{d}}_{oa} h + \\\\left( \\\\J_{oa} \\\\ddq +
\\\\dJ_{oa} \\\\dq \\\\right) \\\\frac{h^2}{2} \\\\end{align*}

where $ \\\\vec{m} $ is a margin vector. So it becomes:

\\\\begin{align*} \\\\A \\\\x + \\\\b &> \\\\vec{0} &
&\\\\Leftrightarrow & \\\\begin{bmatrix} - \\\\J_{oa} \\\\end{bmatrix}
\\\\ddq + \\\\begin{bmatrix} - \\\\dJ_{oa} \\\\dq + 2 \\\\left(
\\\\vec{d}_{oa} - \\\\vec{m} - \\\\dot{\\\\vec{d}}_{oa} h \\\\right) /
h^2 \\\\end{bmatrix} & > \\\\vec{0} \\\\end{align*}

There is some similarity with the orcisir::JointLimitFunction because
we constrain an estimated future state. The same issues arise (it only
constrain the final point, no middle points), so it is interesting to
look at the inflexion point. In the same manner, the time of inflexion
for a constant acceleration such as the inflexion point is on 0 is
computed as follows:

\\\\begin{align*} t_{max} &= 2*(\\\\vec{d}_{oa} - \\\\vec{m}) /
\\\\dot{\\\\vec{d}}_{oa} & & \\\\text{(element-wise operations)}
\\\\end{align*}

For each dof (each line $ i $ ), we test where is the time of
inflexion. If $ 0 < t_{max}[i] < h$, then the inflexion point is in
the horizon of time, we should consider to constrain the motion of
this contact avoidance:

\\\\begin{align*} \\\\ddot{\\\\vec{d}}_{max}[i] &= \\\\frac{
\\\\dot{\\\\vec{d}}_{oa}[i]^2 }{ 2( \\\\vec{d}_{oa}[i] - \\\\vec{m}[i]
) } & & \\\\Rightarrow & \\\\begin{bmatrix} - \\\\J_{oa}
\\\\end{bmatrix} [i] \\\\ddq + \\\\ddot{\\\\vec{d}}_{max}[i] > 0
\\\\end{align*}

When all these constraints have been defined, we select the tightest
ones for each dof.

C++ includes: ContactAvoidanceConstraint.h ";

%feature("docstring")
orcisir::ContactAvoidanceFunction::ContactAvoidanceFunction "ContactAvoidanceFunction::ContactAvoidanceFunction(const orc::Model
&m, orc::Variable &var)

Initialize the contact avoidance constraint function.

Parameters:
-----------

model:  The xde model on which we will get the dynamic parameters

var:  The problem variable that will be used to write this constraint

This class is a generic class to compute matrices for the contact
avoidance function, but it should not be used. You would rather choose
one of the following derivative classes, depending on the choosen
formalism, either full $ \\\\x = [ \\\\ddq \\\\; \\\\torque \\\\;
\\\\force_c ] $ or reduced $ \\\\x = [ \\\\torque \\\\; \\\\force_c ]
$:

orcisir::FullContactAvoidanceFunction

orcisir::ReducedContactAvoidanceFunction ";

%feature("docstring")
orcisir::ContactAvoidanceFunction::~ContactAvoidanceFunction "ContactAvoidanceFunction::~ContactAvoidanceFunction()

Destructor ";

%feature("docstring")
orcisir::ContactAvoidanceFunction::getHorizonOfPrediction "double
ContactAvoidanceFunction::getHorizonOfPrediction() const

Get the time horizon of prediction $ h $ for the contact avoidance
function.

The time horizon (s) ";

%feature("docstring")
orcisir::ContactAvoidanceFunction::setHorizonOfPrediction "void
ContactAvoidanceFunction::setHorizonOfPrediction(double newHpos)

Set the time horizon of prediction $ h $ for the contact avoidance
function.

Parameters:
-----------

newHpos:  The new time horizon (s) ";

%feature("docstring")  orcisir::ContactAvoidanceFunction::getMargin "double ContactAvoidanceFunction::getMargin() const

Get the obstacle avoidance margin $ \\\\vec{m} $.

The margin vector ";

%feature("docstring")  orcisir::ContactAvoidanceFunction::setMargin "void ContactAvoidanceFunction::setMargin(double newMargin)

Set the obstacle avoidance margin $ \\\\vec{m} $.

Parameters:
-----------

newMargin:  The margin vector ";

%feature("docstring")
orcisir::ContactAvoidanceFunction::updateContactInformation "void
ContactAvoidanceFunction::updateContactInformation(const
Eigen::MatrixXd &_JObst, const Eigen::VectorXd &_dJdqOst, const
Eigen::VectorXd &_distObst, const Eigen::VectorXd &_velObst)

Update contact information to compute obstacle avoidance function.

Parameters:
-----------

_JObst:  The Jacobian of obstacle avoidance

_dJdqObst:  The derivative of the Jacobian of obstacle avoidance
multiplied by generalized velocity $ = \\\\dJ_{Obst} \\\\dq $

_distObst:  The relative distance of obstacle avoidance

_velObst:  The relative velocity of obstacle avoidance ";


// File: classorcisir_1_1_fc_quadratic_function.xml
%feature("docstring") orcisir::FcQuadraticFunction "";

%feature("docstring")
orcisir::FcQuadraticFunction::FcQuadraticFunction "orcisir::FcQuadraticFunction::FcQuadraticFunction(orc::Variable &x) ";

%feature("docstring")
orcisir::FcQuadraticFunction::~FcQuadraticFunction "virtual
orcisir::FcQuadraticFunction::~FcQuadraticFunction() ";

%feature("docstring")
orcisir::FcQuadraticFunction::doUpdateInputSizeBegin "void
orcisir::FcQuadraticFunction::doUpdateInputSizeBegin() ";

%feature("docstring")  orcisir::FcQuadraticFunction::updateHessian "void orcisir::FcQuadraticFunction::updateHessian() const ";

%feature("docstring")  orcisir::FcQuadraticFunction::updateq "void
orcisir::FcQuadraticFunction::updateq() const ";

%feature("docstring")  orcisir::FcQuadraticFunction::updater "void
orcisir::FcQuadraticFunction::updater() const ";


// File: classorcisir_1_1_full_contact_avoidance_function.xml
%feature("docstring") orcisir::FullContactAvoidanceFunction "

Create a linear function that represents the contact avoidance
function for the full formalism.

See orcisir::ContactAvoidanceFunction for more information.

C++ includes: ContactAvoidanceConstraint.h ";

%feature("docstring")
orcisir::FullContactAvoidanceFunction::FullContactAvoidanceFunction "FullContactAvoidanceFunction::FullContactAvoidanceFunction(const
orc::Model &model)

Initialize a contact avoidance function designed for the full
formalism.

Parameters:
-----------

model:  The xde model on which we will update the dynamic parameters
";

%feature("docstring")
orcisir::FullContactAvoidanceFunction::~FullContactAvoidanceFunction "FullContactAvoidanceFunction::~FullContactAvoidanceFunction()

Destructor ";


// File: classorcisir_1_1_full_joint_limit_function.xml
%feature("docstring") orcisir::FullJointLimitFunction "

Create a linear function that represents the joint limit function for
the full formalism.

See orcisir::JointLimitFunction ofr more information.

C++ includes: JointLimitConstraint.h ";

%feature("docstring")
orcisir::FullJointLimitFunction::FullJointLimitFunction "FullJointLimitFunction::FullJointLimitFunction(const orc::Model
&model)

Initialize a joint limits function designed for the full formalism.

Parameters:
-----------

model:  The xde model on which we will update the dynamic parameters

It is connected with the model and invalidates $ \\\\b $ when
orc::EVT_CHANGE_VALUE is raised. Furthermore, it computes the Jacobian
matrix $ \\\\A $ which is constant. ";

%feature("docstring")
orcisir::FullJointLimitFunction::~FullJointLimitFunction "FullJointLimitFunction::~FullJointLimitFunction()

Destructor

It is disconnected from the model. ";


// File: classorcisir_1_1_i_s_i_r_constraint.xml
%feature("docstring") orcisir::ISIRConstraint "C++ includes:
ISIRConstraint.h ";

%feature("docstring")  orcisir::ISIRConstraint::ISIRConstraint "orcisir::ISIRConstraint::ISIRConstraint() ";

%feature("docstring")  orcisir::ISIRConstraint::~ISIRConstraint "virtual orcisir::ISIRConstraint::~ISIRConstraint() ";

%feature("docstring")  orcisir::ISIRConstraint::getConstraint "orc::LinearConstraint& orcisir::ISIRConstraint::getConstraint() ";


// File: classorcisir_1_1_i_s_i_r_controller.xml
%feature("docstring") orcisir::ISIRController "

ISIR Controller based on LQP solver for the xde framework.

C++ includes: ISIRController.h ";

%feature("docstring")  orcisir::ISIRController::ISIRController "orcisir::ISIRController::ISIRController(const std::string &ctrlName,
Model &innerModel, ISIRSolver &innerSolver, bool useReducedProblem)

Initialize ISIR controller.

Parameters:
-----------

ctrlName:  The name of the controller

innerModel:  The internal model of the robot one wants to control

innerSolver:  The internal solver one wants to use to make the
quadratic optimization

useReducedProblem:  Tell if the redundant problem is considered
(unknown variable is $ [ \\\\ddq \\\\; \\\\torque \\\\; \\\\force_c ]
$), or is the reduced problem (non-redundant) is considred (unknown
variable is $ [ \\\\torque \\\\; \\\\force_c ] $) ";

%feature("docstring")  orcisir::ISIRController::~ISIRController "orcisir::ISIRController::~ISIRController()

Destructor ";

%feature("docstring")  orcisir::ISIRController::getModel "Model &
orcisir::ISIRController::getModel()

the inner model used to construct this controller instance ";

%feature("docstring")  orcisir::ISIRController::getSolver "ISIRSolver
& orcisir::ISIRController::getSolver()

the inner solver used to construct this controller instance ";

%feature("docstring")  orcisir::ISIRController::isUsingReducedProblem
"bool orcisir::ISIRController::isUsingReducedProblem()

true if variable of reduced problem ( $ [ \\\\torque \\\\; \\\\force_c
] $) is considered ";

%feature("docstring")
orcisir::ISIRController::setVariableMinimizationWeights "void
orcisir::ISIRController::setVariableMinimizationWeights(double w_ddq,
double w_tau, double w_fc) ";

%feature("docstring")  orcisir::ISIRController::takeIntoAccountGravity
"void orcisir::ISIRController::takeIntoAccountGravity(bool useGrav)
";

%feature("docstring")
orcisir::ISIRController::writePerformanceInStream "void
orcisir::ISIRController::writePerformanceInStream(std::ostream
&myOstream, bool addCommaAtEnd) const

Write information about controller performances in a string stream.

Parameters:
-----------

outstream:  the output stream where to write the performances
information

addCommaAtEnd:  If true, add a comma at the end of the stream. If
false, it means that this is the end of the json file, nothing will be
added after that, no comma is added.

See orcisir::Orocos_ISIRController::getPerformances() to know more.
Here it saves:

controller_update_tasks

controller_solve_problem ";

%feature("docstring")  orcisir::ISIRController::getPerformances "std::string orcisir::ISIRController::getPerformances() const

Get information about performances through a string.

Information are saved in a JSON way (http://www.json.org/). It returns
a of dictionnary on the form:

where performance_info are:

controller_update_tasks

controller_solve_problem

solver_prepare

solver_solve

See orcisir::ISIRController::writePerformanceInStream(std::ostream&,
bool) and orcisir::ISIRSolver::writePerformanceInStream(std::ostream&,
bool). ";

%feature("docstring")  orcisir::ISIRController::addConstraint "void
orcisir::ISIRController::addConstraint(orc::LinearConstraint
&constraint) const ";

%feature("docstring")  orcisir::ISIRController::removeConstraint "void orcisir::ISIRController::removeConstraint(orc::LinearConstraint
&constraint) const ";

%feature("docstring")  orcisir::ISIRController::addConstraint "void
orcisir::ISIRController::addConstraint(ISIRConstraint &constraint)
const ";

%feature("docstring")  orcisir::ISIRController::removeConstraint "void orcisir::ISIRController::removeConstraint(ISIRConstraint
&constraint) const ";

%feature("docstring")  orcisir::ISIRController::createISIRTask "ISIRTask & orcisir::ISIRController::createISIRTask(const std::string
&name, const Feature &feature, const Feature &featureDes) const ";

%feature("docstring")  orcisir::ISIRController::createISIRTask "ISIRTask & orcisir::ISIRController::createISIRTask(const std::string
&name, const Feature &feature) const ";

%feature("docstring")  orcisir::ISIRController::createISIRContactTask
"ISIRTask & orcisir::ISIRController::createISIRContactTask(const
std::string &name, const PointContactFeature &feature, double mu,
double margin) const ";


// File: class_isir_debug_trace.xml
%feature("docstring") IsirDebugTrace "C++ includes: ISIRDebug.h ";

%feature("docstring")  IsirDebugTrace::IsirDebugTrace "IsirDebugTrace::IsirDebugTrace(std::ostream &os) ";


// File: classorcisir_1_1_i_s_i_r_dynamic_function.xml
%feature("docstring") orcisir::ISIRDynamicFunction "

Create a linear function that represents the dynamic equation of
motion.

The equation of motion is: \\\\[ \\\\M \\\\ddq + \\\\n + \\\\g = S
\\\\torque - \\\\J_c\\\\tp \\\\force_c \\\\]

So given the variable of our problem $ \\\\x = \\\\begin{bmatrix}
\\\\ddq\\\\tp & \\\\torque\\\\tp &
\\\\force_c\\\\tp\\\\end{bmatrix}\\\\tp $ It returns an equation of
the form:

\\\\begin{align*} \\\\A \\\\x + \\\\b &= \\\\vec{0} &
&\\\\Leftrightarrow & \\\\begin{bmatrix} \\\\M & -S & \\\\J_c\\\\tp
\\\\end{bmatrix} . \\\\begin{bmatrix} \\\\ddq \\\\\\\\ \\\\torque
\\\\\\\\ \\\\force_c\\\\end{bmatrix} + [ \\\\n + \\\\g ] = \\\\vec{0}
\\\\end{align*}

C++ includes: ISIRConstraint.h ";

%feature("docstring")
orcisir::ISIRDynamicFunction::ISIRDynamicFunction "orcisir::ISIRDynamicFunction::ISIRDynamicFunction(const orc::Model
&model)

Initialize a dynamic equation of motion function.

Parameters:
-----------

model:  The xde model on which we will update the dynamic parameters

It is connected with the model on \"orc::EVT_CHANGE_VALUE\", meaning
that a,y modification in the model will invalidate this linear
function. ";

%feature("docstring")
orcisir::ISIRDynamicFunction::~ISIRDynamicFunction "orcisir::ISIRDynamicFunction::~ISIRDynamicFunction()

Destructor ";

%feature("docstring")
orcisir::ISIRDynamicFunction::takeIntoAccountGravity "void
orcisir::ISIRDynamicFunction::takeIntoAccountGravity(bool useGrav) ";

%feature("docstring")
orcisir::ISIRDynamicFunction::getInertiaMatrixInverseJchiT "const
Eigen::MatrixXd &
orcisir::ISIRDynamicFunction::getInertiaMatrixInverseJchiT() const ";

%feature("docstring")
orcisir::ISIRDynamicFunction::getInertiaMatrixInverseLinNonLinGrav "const Eigen::VectorXd &
orcisir::ISIRDynamicFunction::getInertiaMatrixInverseLinNonLinGrav()
const ";

%feature("docstring")  orcisir::ISIRDynamicFunction::getActionVariable
"orc::Variable & orcisir::ISIRDynamicFunction::getActionVariable()
const ";


// File: classorcisir_1_1_i_s_i_r_solver.xml
%feature("docstring") orcisir::ISIRSolver "

A generic abstract class the solvers that can be used in the ISIR
Controller.

It is based on quadratic solvers.

To get a concrete implementation of ISIR solvers, you should call:

orcisir::OneLevelSolver if we consider that all the tasks registered
have the same level of importance (but not necessarily the same
weights)

orcisir::HierarchySolver if we consider that tasks registered can have
different level of importance

C++ includes: ISIRSolver.h ";

%feature("docstring")  orcisir::ISIRSolver::ISIRSolver "ISIRSolver::ISIRSolver()

Initialize an abstract ISIR Solver. ";

%feature("docstring")  orcisir::ISIRSolver::~ISIRSolver "ISIRSolver::~ISIRSolver()

Destructor ";

%feature("docstring")  orcisir::ISIRSolver::printValuesAtSolution "void ISIRSolver::printValuesAtSolution()

I don't really know, I suppose to print internal values when solution
is found.

Actually does nothing. ";

%feature("docstring")  orcisir::ISIRSolver::toString "std::string
ISIRSolver::toString() const

I don't really know, I support to write the problem as a string.

Actually does nothing. ";

%feature("docstring")  orcisir::ISIRSolver::addObjective "void
ISIRSolver::addObjective(orc::QuadraticObjective &obj)

Add a quadratic objective to the controller.

Parameters:
-----------

obj:  The quadratic objective to add

This objective is saved in an internal vector of objectives. ";

%feature("docstring")  orcisir::ISIRSolver::removeObjective "void
ISIRSolver::removeObjective(orc::QuadraticObjective &obj)

Remove a quadratic objective to the controller.

Parameters:
-----------

obj:  The quadratic objective to remove

This objective is removed from the internal vector of objectives. ";

%feature("docstring")  orcisir::ISIRSolver::setObjectiveLevel "virtual void
orcisir::ISIRSolver::setObjectiveLevel(orc::QuadraticObjective &obj,
int level)=0 ";

%feature("docstring")  orcisir::ISIRSolver::addConstraint "void
ISIRSolver::addConstraint(orc::LinearConstraint &constraint)

Add a linear constraint to the controller.

Parameters:
-----------

constraint:  The linear constraint to add

This constraint is saved in an internal vector of equality or
inequality constraints, depending on the type of constraint. ";

%feature("docstring")  orcisir::ISIRSolver::removeConstraint "void
ISIRSolver::removeConstraint(orc::LinearConstraint &constraint)

Removea linear constraint to the controller.

Parameters:
-----------

constraint:  The linear constraint to remove ";

%feature("docstring")  orcisir::ISIRSolver::writePerformanceInStream "void ISIRSolver::writePerformanceInStream(std::ostream &myOstream,
bool addCommaAtEnd)

Write information about solver performances in a string stream.

Parameters:
-----------

outstream:  the output stream where to write the performances
information

addCommaAtEnd:  If true, add a comma at the end of the stream. If
false, it means that this is the end of the json file, nothing will be
added after that, no comma is added.

See orcisir::Orocos_ISIRController::getPerformances() to know more.
Here it saves:

solver_prepare

solver_solve ";


// File: classorcisir_1_1_i_s_i_r_task.xml
%feature("docstring") orcisir::ISIRTask "

A generic abstract task for the ISIR controller.

The main difference with the Task class defined in the xde framework
is the addition of a level parameter. Hence, a hierarchical set of
tasks can be solved.

This level information may have been added to the controller or the
solver, not directly added to the task, and the Task class of the xde
framework may have been used instead of this new class. But I think
the level is the same concept as the weight (an importance), so I add
it direcly in the task class, like the weight. Furthermore, writting
this class helps me to better understand the xde framework.  Concrete
class are orcisir::ISIRFullTask and orcisir::ISIRReducedTask .

C++ includes: ISIRTask.h ";

%feature("docstring")  orcisir::ISIRTask::ISIRTask "orcisir::ISIRTask::ISIRTask(const std::string &taskName, const Model
&innerModel, const Feature &feature, const Feature &featureDes)

Initialize a new ISIR Task.

Parameters:
-----------

name:  The name of the task

model:  The xde model on which we will update the dynamic parameters

feature:  The task feature, meaning what we want to control

featureDes:  The desired task feature, meaning the goal we want to
reach with the feature ";

%feature("docstring")  orcisir::ISIRTask::ISIRTask "orcisir::ISIRTask::ISIRTask(const std::string &taskName, const Model
&innerModel, const Feature &feature)

Initialize a new ISIR Task.

Parameters:
-----------

name:  The name of the task

model:  The xde model on which we will update the dynamic parameters

feature:  The task feature, meaning what we want to control ";

%feature("docstring")  orcisir::ISIRTask::~ISIRTask "orcisir::ISIRTask::~ISIRTask() ";

%feature("docstring")  orcisir::ISIRTask::initAsAccelerationTask "void orcisir::ISIRTask::initAsAccelerationTask() ";

%feature("docstring")  orcisir::ISIRTask::initAsTorqueTask "void
orcisir::ISIRTask::initAsTorqueTask() ";

%feature("docstring")  orcisir::ISIRTask::initAsForceTask "void
orcisir::ISIRTask::initAsForceTask() ";

%feature("docstring")  orcisir::ISIRTask::getTaskType "ISIRTask::TYPETASK orcisir::ISIRTask::getTaskType() const ";

%feature("docstring")  orcisir::ISIRTask::getComputedForce "const
Eigen::VectorXd & orcisir::ISIRTask::getComputedForce() const ";


// File: classorcisir_1_1_joint_limit_constraint.xml
%feature("docstring") orcisir::JointLimitConstraint "C++ includes:
JointLimitConstraint.h ";

%feature("docstring")
orcisir::JointLimitConstraint::JointLimitConstraint "JointLimitConstraint::JointLimitConstraint(const orc::Model &model,
double hpos=.2) ";

%feature("docstring")
orcisir::JointLimitConstraint::JointLimitConstraint "JointLimitConstraint::JointLimitConstraint(const orc::Model &model,
const Eigen::VectorXd &lowerLimits, const Eigen::VectorXd
&upperLimits, double hpos=.2) ";

%feature("docstring")
orcisir::JointLimitConstraint::~JointLimitConstraint "virtual
orcisir::JointLimitConstraint::~JointLimitConstraint() ";

%feature("docstring")
orcisir::JointLimitConstraint::getHorizonOfPrediction "double
JointLimitConstraint::getHorizonOfPrediction() const ";

%feature("docstring")
orcisir::JointLimitConstraint::setHorizonOfPrediction "void
JointLimitConstraint::setHorizonOfPrediction(double newHpos) ";

%feature("docstring")  orcisir::JointLimitConstraint::setJointLimits "void JointLimitConstraint::setJointLimits(const Eigen::VectorXd
&lowerLimits, const Eigen::VectorXd &upperLimits) ";

%feature("docstring")
orcisir::JointLimitConstraint::setJointLowerLimits "void
JointLimitConstraint::setJointLowerLimits(const Eigen::VectorXd
&newLowerLimits) ";

%feature("docstring")
orcisir::JointLimitConstraint::setJointUpperLimits "void
JointLimitConstraint::setJointUpperLimits(const Eigen::VectorXd
&newUpperLimits) ";

%feature("docstring")
orcisir::JointLimitConstraint::getJointLowerLimits "const
Eigen::VectorXd & JointLimitConstraint::getJointLowerLimits() const ";

%feature("docstring")
orcisir::JointLimitConstraint::getJointUpperLimits "const
Eigen::VectorXd & JointLimitConstraint::getJointUpperLimits() const ";

%feature("docstring")  orcisir::JointLimitConstraint::setJointLimit "void JointLimitConstraint::setJointLimit(int i, double newLowerLimit,
double newUpperLimit) ";

%feature("docstring")
orcisir::JointLimitConstraint::setJointLowerLimit "void
JointLimitConstraint::setJointLowerLimit(int i, double newLowerLimit)
";

%feature("docstring")
orcisir::JointLimitConstraint::setJointUpperLimit "void
JointLimitConstraint::setJointUpperLimit(int i, double newUpperLimit)
";

%feature("docstring")
orcisir::JointLimitConstraint::getJointLowerLimit "double
JointLimitConstraint::getJointLowerLimit(int i) const ";

%feature("docstring")
orcisir::JointLimitConstraint::getJointUpperLimit "double
JointLimitConstraint::getJointUpperLimit(int i) const ";


// File: classorcisir_1_1_joint_limit_function.xml
%feature("docstring") orcisir::JointLimitFunction "

Create a linear function that represents the joint limit function.

The joint limit function $ \\\\q_{min} < \\\\q < \\\\q{max} $ defined
in this controller is composed of a couple of inequalities:

the first one is computed on a horizon h. The system brakes far from
the limit, but can pass inside the constraint when close to it
(because it only constrains the final point, not the middle ones).

the second one is computed with the time of inflexion. The system
brakes when close to the limits, but some singularities appears when
it is on the limits.

The two computation have their drawbacks, but together they allow
limits avoidance properly.

Joint limit constraint if inflexion point is not taken into account
The first one which constrains the last point over the horizon h can
be expressed as follows (depending on $ \\\\ddq $):

\\\\begin{align*} \\\\q_{min} < \\\\q + \\\\dq h + \\\\ddq
\\\\frac{h^2}{2} < \\\\q_{max} \\\\end{align*}

\\\\begin{align*} A \\\\x + \\\\b &> \\\\vec{0} & &\\\\Leftrightarrow
& \\\\begin{bmatrix} - \\\\Id{} \\\\\\\\ \\\\Id{} \\\\end{bmatrix}
\\\\ddq + 2 \\\\begin{bmatrix} \\\\q_{max} - \\\\q + \\\\dq h \\\\\\\\
\\\\q_{min} - \\\\q + \\\\dq h \\\\end{bmatrix} / h^2 &> \\\\vec{0}
\\\\end{align*}

The second one is more \"tricky\". First it computes the times of
inflexion for a constant acceleration such as the inflexion point is
on 0:

\\\\begin{align*} t_{max} &= - 2 ( \\\\q - \\\\q_{max} ) / \\\\dq &
&\\\\text{(element-wise operations)} \\\\\\\\ t_{min} &= - 2 ( \\\\q -
\\\\q_{min} ) / \\\\dq \\\\end{align*}

For each dof (each line $ i $ ), we test where is the time of
inflexion. If $ 0 < t_{max}[i] < h$, then the inflexion point is in
the horizon of time, we should consider to constrain the motion of
this dof:

\\\\begin{align*} \\\\ddq_{max}[i] &= \\\\frac{ \\\\dq[i]^2 }{
2(\\\\q[i] - \\\\q_{max}[i] ) } & & \\\\Rightarrow & \\\\ddq [i] +
\\\\ddq_{max}[i] > 0 \\\\end{align*}

Again, if $ 0 < t_{min}[i] < h $ :

\\\\begin{align*} \\\\ddq_{min}[i] &= \\\\frac{ \\\\dq[i]^2 }{
2(\\\\q[i] - \\\\q_{min}[i] ) } & & \\\\Rightarrow & \\\\ddq [i] +
\\\\ddq_{min}[i] > 0 \\\\end{align*}

When all these constraints have been defined, we select the tightest
ones for each dof.

Explaination of the inflexion point in the joint limit constraint

C++ includes: JointLimitConstraint.h ";

%feature("docstring")  orcisir::JointLimitFunction::JointLimitFunction
"JointLimitFunction::JointLimitFunction(const orc::Model &m,
orc::Variable &var)

Initialize the joint limit constraint function.

Parameters:
-----------

model:  The xde model on which we will get the dynamic parameters

var:  The problem variable that will be used to write this constraint

This class is a generic class to compute matrices for the joint limit
function, but it should not be used. You would rather choose one of
the following derivative classes, depending on the choosen formalism,
either full $ \\\\x = [ \\\\ddq \\\\; \\\\torque \\\\; \\\\force_c ] $
or reduced $ \\\\x = [ \\\\torque \\\\; \\\\force_c ] $:

orcisir::FullJointLimitFunction

orcisir::ReducedJointLimitFunction ";

%feature("docstring")
orcisir::JointLimitFunction::~JointLimitFunction "JointLimitFunction::~JointLimitFunction()

Destructor ";

%feature("docstring")
orcisir::JointLimitFunction::getHorizonOfPrediction "double
JointLimitFunction::getHorizonOfPrediction() const

Get the time horizon of prediction $ h $ for the joint limit function.

The time horizon (s) ";

%feature("docstring")
orcisir::JointLimitFunction::setHorizonOfPrediction "void
JointLimitFunction::setHorizonOfPrediction(double newHpos)

Set the time horizon of prediction $ h $ for the joint limit function.

Parameters:
-----------

newHpos:  The new time horizon (s) ";

%feature("docstring")  orcisir::JointLimitFunction::setJointLimits "void JointLimitFunction::setJointLimits(const Eigen::VectorXd
&lowerLimits, const Eigen::VectorXd &upperLimits)

Set the joint limits $ \\\\q_{min}, \\\\q_{max} $.

Parameters:
-----------

lowerLimits:  The lower bound

upperLimits:  The upper bound ";

%feature("docstring")
orcisir::JointLimitFunction::setJointLowerLimits "void
JointLimitFunction::setJointLowerLimits(const Eigen::VectorXd
&newLowerLimits)

Set the joint limit $ \\\\q_{min} $.

Parameters:
-----------

newLowerLimits:  The lower bound ";

%feature("docstring")
orcisir::JointLimitFunction::setJointUpperLimits "void
JointLimitFunction::setJointUpperLimits(const Eigen::VectorXd
&newUpperLimits)

Set the joint limits $ \\\\q_{max} $.

Parameters:
-----------

newUpperLimits:  The upper bound ";

%feature("docstring")
orcisir::JointLimitFunction::getJointLowerLimits "const
Eigen::VectorXd & JointLimitFunction::getJointLowerLimits() const

Get the joint limit $ \\\\q_{min} $.

The lower bound ";

%feature("docstring")
orcisir::JointLimitFunction::getJointUpperLimits "const
Eigen::VectorXd & JointLimitFunction::getJointUpperLimits() const

Get the joint limit $ \\\\q_{max} $.

The upper bound ";

%feature("docstring")  orcisir::JointLimitFunction::setJointLimit "void JointLimitFunction::setJointLimit(int i, double newLowerLimit,
double newUpperLimit)

Set the joint limits for one dof $ \\\\q_{min}[i], \\\\q_{max}[i] $.

Parameters:
-----------

i:  The dof index whose bounds are modified

newLowerLimit:  The lower bound

newUpperLimit:  The upper bound ";

%feature("docstring")  orcisir::JointLimitFunction::setJointLowerLimit
"void JointLimitFunction::setJointLowerLimit(int i, double
newLowerLimit)

Set the joint limit for one dof $ \\\\q_{min}[i] $.

Parameters:
-----------

i:  The dof index whose bound is modified

newLowerLimit:  The lower bound ";

%feature("docstring")  orcisir::JointLimitFunction::setJointUpperLimit
"void JointLimitFunction::setJointUpperLimit(int i, double
newUpperLimit)

Set the joint limit for one dof $ \\\\q_{max}[i] $.

Parameters:
-----------

i:  The dof index whose bound is modified

newUpperLimit:  The upper bound ";

%feature("docstring")  orcisir::JointLimitFunction::getJointLowerLimit
"double JointLimitFunction::getJointLowerLimit(int i) const

Get the joint limit for one dof $ \\\\q_{min}[i] $.

Parameters:
-----------

i:  The dof index

The lower bound ";

%feature("docstring")  orcisir::JointLimitFunction::getJointUpperLimit
"double JointLimitFunction::getJointUpperLimit(int i) const

Get the joint limit for one dof $ \\\\q_{max}[i] $.

Parameters:
-----------

i:  The dof index

The upper bound ";


// File: classorcisir_1_1_one_level_solver.xml
%feature("docstring") orcisir::OneLevelSolver "

Abstract solver class that only consider one level of importance for
all tasks.

Concrete instances are orcisir::OneLevelSolverWithQuadProg and
orcisir::OneLevelSolverWithQLD .

C++ includes: OneLevelSolver.h ";

%feature("docstring")  orcisir::OneLevelSolver::OneLevelSolver "OneLevelSolver::OneLevelSolver()

Constructor of the abstract one level solver.

Parameters:
-----------

m:  The Model of the robot ";

%feature("docstring")  orcisir::OneLevelSolver::~OneLevelSolver "OneLevelSolver::~OneLevelSolver()

Destructor ";

%feature("docstring")  orcisir::OneLevelSolver::setObjectiveLevel "void OneLevelSolver::setObjectiveLevel(orc::QuadraticObjective &obj,
int level)

Set the level of a particular objective registered in the solver.

Parameters:
-----------

obj:  The objective instance with a new level

level:  the new level

Actually, as this solver considers that all tasks have the same level,
this function does nothing. ";

%feature("docstring")  orcisir::OneLevelSolver::toString "std::string
OneLevelSolver::toString() const

I don't really know, I support to write the problem as a string.

Actually does nothing. ";


// File: classorcisir_1_1_one_level_solver_with_q_l_d.xml
%feature("docstring") orcisir::OneLevelSolverWithQLD "

Solver class that only consider one level of importance for all tasks
using QLD.

It uses a linear quadratic program which is included in the xde
framework.

QLD solve the following problem:

\\\\begin{align*} \\\\argmin{\\\\x} &: \\\\; \\\\frac{1}{2}
\\\\x\\\\tp P \\\\x + \\\\vec{q}\\\\tp \\\\x \\\\\\\\ & \\\\A \\\\x +
\\\\b \\\\geqq \\\\vec{0} \\\\\\\\ & \\\\x_{min} \\\\leq \\\\x \\\\leq
\\\\x_{max} \\\\end{align*}

with \\\\begin{align*} \\\\A \\\\x + \\\\b &\\\\geqq \\\\vec{0} &
&\\\\Leftrightarrow &\\\\begin{bmatrix} CE \\\\\\\\ CI
\\\\end{bmatrix} \\\\x + \\\\begin{bmatrix} \\\\vec{ce}_0 \\\\\\\\
\\\\vec{ci}_0 \\\\end{bmatrix} & \\\\quad \\\\begin{matrix} = \\\\\\\\
\\\\geq \\\\end{matrix} \\\\quad \\\\begin{bmatrix} \\\\vec{0}
\\\\\\\\ \\\\vec{0} \\\\end{bmatrix} \\\\end{align*}

C++ includes: OneLevelSolver.h ";

%feature("docstring")
orcisir::OneLevelSolverWithQLD::OneLevelSolverWithQLD "OneLevelSolverWithQLD::OneLevelSolverWithQLD()

Instanciate a concrete one level solver with QLD.

Parameters:
-----------

m:  The Model of the robot ";

%feature("docstring")
orcisir::OneLevelSolverWithQLD::~OneLevelSolverWithQLD "OneLevelSolverWithQLD::~OneLevelSolverWithQLD()

Destructor ";


// File: classorcisir_1_1_one_level_solver_with_quad_prog.xml
%feature("docstring") orcisir::OneLevelSolverWithQuadProg "

Solver class that only consider one level of importance for all tasks
using Quadprog++.

It uses a linear quadratic program which comes
fromhttp://quadprog.sourceforge.net/ .

Quadprog++ solve the following problem:

\\\\begin{align*} \\\\argmin{\\\\x} &: \\\\; \\\\frac{1}{2}
\\\\x\\\\tp G \\\\x + \\\\vec{g}_0\\\\tp \\\\x \\\\\\\\ & CE \\\\x +
\\\\vec{ce}_0 = \\\\vec{0} \\\\\\\\ & CI \\\\x + \\\\vec{ci}_0 \\\\geq
\\\\vec{0} \\\\end{align*}

C++ includes: OneLevelSolver.h ";

%feature("docstring")
orcisir::OneLevelSolverWithQuadProg::OneLevelSolverWithQuadProg "OneLevelSolverWithQuadProg::OneLevelSolverWithQuadProg()

Instanciate a concrete one level solver with Quadprog++.

Parameters:
-----------

m:  The Model of the robot ";

%feature("docstring")
orcisir::OneLevelSolverWithQuadProg::~OneLevelSolverWithQuadProg "OneLevelSolverWithQuadProg::~OneLevelSolverWithQuadProg()

Destructor ";


// File: classorcisir_1_1_partial_model_state.xml
%feature("docstring") orcisir::PartialModelState "

A partial state of the model.

It get information about the partial state from a Model.

C++ includes: ISIRPartialState.h ";

%feature("docstring")  orcisir::PartialModelState::PartialModelState "orcisir::PartialModelState::PartialModelState(const std::string &name,
const Model &model, const Eigen::VectorXi &selectedDofs, int
whichPart) ";

%feature("docstring")  orcisir::PartialModelState::~PartialModelState
"orcisir::PartialModelState::~PartialModelState() ";

%feature("docstring")  orcisir::PartialModelState::getInertiaMatrix "const MatrixXd & orcisir::PartialModelState::getInertiaMatrix() const
";

%feature("docstring")
orcisir::PartialModelState::getInertiaMatrixInverse "const MatrixXd &
orcisir::PartialModelState::getInertiaMatrixInverse() const ";

%feature("docstring")  orcisir::PartialModelState::q "const
Eigen::VectorXd & orcisir::PartialModelState::q() const ";

%feature("docstring")  orcisir::PartialModelState::qdot "const
Eigen::VectorXd & orcisir::PartialModelState::qdot() const ";

%feature("docstring")  orcisir::PartialModelState::qddot "const
Eigen::VectorXd & orcisir::PartialModelState::qddot() const ";

%feature("docstring")  orcisir::PartialModelState::tau "const
Eigen::VectorXd & orcisir::PartialModelState::tau() const ";


// File: classorcisir_1_1_partial_state.xml
%feature("docstring") orcisir::PartialState "

A abstract partial state.

This class is greatly inspired from the FullState class defined in the
xde framework.

C++ includes: ISIRPartialState.h ";

%feature("docstring")  orcisir::PartialState::PartialState "orcisir::PartialState::PartialState(const std::string &name, const
Model &model, const Eigen::VectorXi &selectedDofs, int whichPart) ";

%feature("docstring")  orcisir::PartialState::~PartialState "orcisir::PartialState::~PartialState()=0 ";

%feature("docstring")  orcisir::PartialState::getModel "const Model &
orcisir::PartialState::getModel() const ";

%feature("docstring")  orcisir::PartialState::getSize "int
orcisir::PartialState::getSize() const ";

%feature("docstring")  orcisir::PartialState::getJacobian "const
MatrixXd & orcisir::PartialState::getJacobian() const ";

%feature("docstring")  orcisir::PartialState::getInertiaMatrix "virtual const Eigen::MatrixXd&
orcisir::PartialState::getInertiaMatrix() const =0 ";

%feature("docstring")  orcisir::PartialState::getInertiaMatrixInverse
"virtual const Eigen::MatrixXd&
orcisir::PartialState::getInertiaMatrixInverse() const =0 ";

%feature("docstring")  orcisir::PartialState::q "virtual const
Eigen::VectorXd& orcisir::PartialState::q() const =0 ";

%feature("docstring")  orcisir::PartialState::qdot "virtual const
Eigen::VectorXd& orcisir::PartialState::qdot() const =0 ";

%feature("docstring")  orcisir::PartialState::qddot "virtual const
Eigen::VectorXd& orcisir::PartialState::qddot() const =0 ";

%feature("docstring")  orcisir::PartialState::tau "virtual const
Eigen::VectorXd& orcisir::PartialState::tau() const =0 ";


// File: classorcisir_1_1_partial_state_feature.xml
%feature("docstring") orcisir::PartialStateFeature "

A partial state feature.

This class is greatly inspired from the FullStateFeature class defined
in the xde framework.

C++ includes: ISIRFeature.h ";

%feature("docstring")
orcisir::PartialStateFeature::PartialStateFeature "orcisir::PartialStateFeature::PartialStateFeature(const std::string
&name, const PartialState &state) ";

%feature("docstring")  orcisir::PartialStateFeature::getDimension "int orcisir::PartialStateFeature::getDimension() const ";

%feature("docstring")  orcisir::PartialStateFeature::getSpaceTransform
"const MatrixXd & orcisir::PartialStateFeature::getSpaceTransform()
const ";

%feature("docstring")  orcisir::PartialStateFeature::computeEffort "const VectorXd & orcisir::PartialStateFeature::computeEffort(const
Feature &featureDes) const ";

%feature("docstring")
orcisir::PartialStateFeature::computeAcceleration "const VectorXd &
orcisir::PartialStateFeature::computeAcceleration(const Feature
&featureDes) const ";

%feature("docstring")  orcisir::PartialStateFeature::computeError "const VectorXd & orcisir::PartialStateFeature::computeError(const
Feature &featureDes) const ";

%feature("docstring")  orcisir::PartialStateFeature::computeErrorDot "const VectorXd & orcisir::PartialStateFeature::computeErrorDot(const
Feature &featureDes) const ";

%feature("docstring")  orcisir::PartialStateFeature::computeJacobian "const MatrixXd & orcisir::PartialStateFeature::computeJacobian(const
Feature &featureDes) const ";

%feature("docstring")
orcisir::PartialStateFeature::computeProjectedMass "const MatrixXd &
orcisir::PartialStateFeature::computeProjectedMass(const Feature
&featureDes) const ";

%feature("docstring")
orcisir::PartialStateFeature::computeProjectedMassInverse "const
MatrixXd &
orcisir::PartialStateFeature::computeProjectedMassInverse(const
Feature &featureDes) const ";

%feature("docstring")  orcisir::PartialStateFeature::computeEffort "const VectorXd & orcisir::PartialStateFeature::computeEffort() const
";

%feature("docstring")
orcisir::PartialStateFeature::computeAcceleration "const VectorXd &
orcisir::PartialStateFeature::computeAcceleration() const ";

%feature("docstring")  orcisir::PartialStateFeature::computeError "const VectorXd & orcisir::PartialStateFeature::computeError() const ";

%feature("docstring")  orcisir::PartialStateFeature::computeErrorDot "const VectorXd & orcisir::PartialStateFeature::computeErrorDot() const
";

%feature("docstring")  orcisir::PartialStateFeature::computeJacobian "const MatrixXd & orcisir::PartialStateFeature::computeJacobian() const
";

%feature("docstring")
orcisir::PartialStateFeature::computeProjectedMass "const MatrixXd &
orcisir::PartialStateFeature::computeProjectedMass() const ";

%feature("docstring")
orcisir::PartialStateFeature::computeProjectedMassInverse "const
MatrixXd & orcisir::PartialStateFeature::computeProjectedMassInverse()
const ";


// File: classorcisir_1_1_partial_target_state.xml
%feature("docstring") orcisir::PartialTargetState "

A target for a model partial state.

It represents a desired partial state.

C++ includes: ISIRPartialState.h ";

%feature("docstring")  orcisir::PartialTargetState::PartialTargetState
"orcisir::PartialTargetState::PartialTargetState(const std::string
&name, const Model &model, const Eigen::VectorXi &selectedDofs, int
whichPart) ";

%feature("docstring")
orcisir::PartialTargetState::~PartialTargetState "orcisir::PartialTargetState::~PartialTargetState() ";

%feature("docstring")  orcisir::PartialTargetState::getInertiaMatrix "const MatrixXd & orcisir::PartialTargetState::getInertiaMatrix() const
";

%feature("docstring")
orcisir::PartialTargetState::getInertiaMatrixInverse "const MatrixXd
& orcisir::PartialTargetState::getInertiaMatrixInverse() const ";

%feature("docstring")  orcisir::PartialTargetState::q "const
Eigen::VectorXd & orcisir::PartialTargetState::q() const ";

%feature("docstring")  orcisir::PartialTargetState::qdot "const
Eigen::VectorXd & orcisir::PartialTargetState::qdot() const ";

%feature("docstring")  orcisir::PartialTargetState::qddot "const
Eigen::VectorXd & orcisir::PartialTargetState::qddot() const ";

%feature("docstring")  orcisir::PartialTargetState::tau "const
Eigen::VectorXd & orcisir::PartialTargetState::tau() const ";

%feature("docstring")  orcisir::PartialTargetState::set_q "void
orcisir::PartialTargetState::set_q(const Eigen::VectorXd &q) ";

%feature("docstring")  orcisir::PartialTargetState::set_qdot "void
orcisir::PartialTargetState::set_qdot(const Eigen::VectorXd &qdot) ";

%feature("docstring")  orcisir::PartialTargetState::set_qddot "void
orcisir::PartialTargetState::set_qddot(const Eigen::VectorXd &qddot)
";

%feature("docstring")  orcisir::PartialTargetState::set_tau "void
orcisir::PartialTargetState::set_tau(const Eigen::VectorXd &tau) ";


// File: classorcisir_1_1_performance_recorder.xml
%feature("docstring") orcisir::PerformanceRecorder "

To get and save time information to collect some data on performances.

It can be used in two ways. First to save a timeline:

Or to save loop performances:

C++ includes: Performances.h ";

%feature("docstring")
orcisir::PerformanceRecorder::PerformanceRecorder "orcisir::PerformanceRecorder::PerformanceRecorder()

Create a performance recorder. It also initializes the internal
initial time. ";

%feature("docstring")
orcisir::PerformanceRecorder::~PerformanceRecorder "virtual
orcisir::PerformanceRecorder::~PerformanceRecorder()

Destructor ";

%feature("docstring")  orcisir::PerformanceRecorder::getCurrentTime "double orcisir::PerformanceRecorder::getCurrentTime()

Get the current time.

It calls method \"gettimeofday\" and returns a value representing the
current time of the day in second. ";

%feature("docstring")  orcisir::PerformanceRecorder::initializeTime "void orcisir::PerformanceRecorder::initializeTime()

Initialize internal Zero time.

It calls method getCurrentTime() to get a time reference. ";

%feature("docstring")  orcisir::PerformanceRecorder::saveRelativeTime
"void orcisir::PerformanceRecorder::saveRelativeTime()

Save the relative elapsed time since the time reference, i.e. the last
call of initializeTime().

The value is saved in a list available with getSavedTime() const ";

%feature("docstring")  orcisir::PerformanceRecorder::getRelativeTime "double orcisir::PerformanceRecorder::getRelativeTime()

get the relative elapsed time since the time reference, i.e. the last
call of initializeTime(). ";

%feature("docstring")  orcisir::PerformanceRecorder::saveTime "void
orcisir::PerformanceRecorder::saveTime(double t) ";

%feature("docstring")  orcisir::PerformanceRecorder::getSavedTime "const std::list< double >&
orcisir::PerformanceRecorder::getSavedTime() const

Get the list where are saved the relative times. ";

%feature("docstring")  orcisir::PerformanceRecorder::writeInStream "void orcisir::PerformanceRecorder::writeInStream(const std::string
&headerName, std::ostream &outstream, bool addCommaAtEnd)

Save performances information in a output stream.

Parameters:
-----------

headerName:  The name of the performance saved in the

outstream:  the output stream where to write the performances
information

addCommaAtEnd:  If true, add a comma at the end of the stream. If
false, it means that this is the end of the json file, nothing will be
added after that, no comma is added. ";


// File: classorcisir_1_1_reduced_contact_avoidance_function.xml
%feature("docstring") orcisir::ReducedContactAvoidanceFunction "

Create a linear function that represents the contact avoidance
function for the reduced formalism.

See orcisir::ContactAvoidanceFunction for more information.

C++ includes: ContactAvoidanceConstraint.h ";

%feature("docstring")  orcisir::ReducedContactAvoidanceFunction::Reduc
edContactAvoidanceFunction "
ReducedContactAvoidanceFunction::ReducedContactAvoidanceFunction(const
orc::Model &model, const ISIRDynamicFunction &dynamicEquation)

Initialize a contact avoidance function designed for the reduced
formalism.

Parameters:
-----------

model:  The xde model on which we will update the dynamic parameters
";

%feature("docstring")  orcisir::ReducedContactAvoidanceFunction::~Redu
cedContactAvoidanceFunction "
ReducedContactAvoidanceFunction::~ReducedContactAvoidanceFunction()

Destructor ";


// File: classorcisir_1_1_reduced_joint_limit_function.xml
%feature("docstring") orcisir::ReducedJointLimitFunction "

Create a linear function that represents the joint limit function for
the reduced formalism.

See orcisir::JointLimitFunction ofr more information.

C++ includes: JointLimitConstraint.h ";

%feature("docstring")
orcisir::ReducedJointLimitFunction::ReducedJointLimitFunction "ReducedJointLimitFunction::ReducedJointLimitFunction(const orc::Model
&model, const ISIRDynamicFunction &dynamicEquation)

Initialize a joint limits function designed for the reduced formalism.

Parameters:
-----------

model:  The xde model on which we will update the dynamic parameters

It is connected with the model and invalidates all when
orc::EVT_CHANGE_VALUE is raised. Furthermore, it computes the Jacobian
matrix $ \\\\A $ for the full formalism which is constant. It will be
transformed to fit the reduced formalism. ";

%feature("docstring")
orcisir::ReducedJointLimitFunction::~ReducedJointLimitFunction "ReducedJointLimitFunction::~ReducedJointLimitFunction()

Destructor

It is disconnected from the model. ";


// File: classorcisir_1_1_torque_limit_constraint.xml
%feature("docstring") orcisir::TorqueLimitConstraint "C++ includes:
TorqueLimitConstraint.h ";

%feature("docstring")
orcisir::TorqueLimitConstraint::TorqueLimitConstraint "orcisir::TorqueLimitConstraint::TorqueLimitConstraint(const orc::Model
&model) ";

%feature("docstring")
orcisir::TorqueLimitConstraint::TorqueLimitConstraint "orcisir::TorqueLimitConstraint::TorqueLimitConstraint(const orc::Model
&model, const Eigen::VectorXd &torqueLimits) ";

%feature("docstring")
orcisir::TorqueLimitConstraint::~TorqueLimitConstraint "virtual
orcisir::TorqueLimitConstraint::~TorqueLimitConstraint() ";

%feature("docstring")  orcisir::TorqueLimitConstraint::setTorqueLimits
"void orcisir::TorqueLimitConstraint::setTorqueLimits(const
Eigen::VectorXd &torqueLimits) ";

%feature("docstring")  orcisir::TorqueLimitConstraint::getTorqueLimits
"const Eigen::VectorXd&
orcisir::TorqueLimitConstraint::getTorqueLimits() const ";


// File: classorcisir_1_1_torque_limit_function.xml
%feature("docstring") orcisir::TorqueLimitFunction "

Create a linear function that represents the torque limit function.

The torque limit inequality is: \\\\[ - \\\\torque_{max} < \\\\torque
< \\\\torque_{max} \\\\]

It returns an equation of the form:

\\\\begin{align*} \\\\A \\\\x + \\\\b &> \\\\vec{0} &
&\\\\Leftrightarrow & \\\\begin{bmatrix} - \\\\Id{} \\\\\\\\ \\\\Id{}
\\\\end{bmatrix} . \\\\torque + \\\\begin{bmatrix} \\\\torque_{max}
\\\\\\\\ \\\\torque_{max} \\\\end{bmatrix} &> \\\\vec{0}
\\\\end{align*}

C++ includes: TorqueLimitConstraint.h ";

%feature("docstring")
orcisir::TorqueLimitFunction::TorqueLimitFunction "orcisir::TorqueLimitFunction::TorqueLimitFunction(const orc::Model
&model)

Initialize a torque limit inequality function.

Parameters:
-----------

model:  The xde model on which we will get the dynamic parameters

It initialize the Jacobian of the linear function, $ \\\\A $ which is
constant. ";

%feature("docstring")
orcisir::TorqueLimitFunction::~TorqueLimitFunction "orcisir::TorqueLimitFunction::~TorqueLimitFunction()

Destructor ";

%feature("docstring")  orcisir::TorqueLimitFunction::setTorqueLimits "void orcisir::TorqueLimitFunction::setTorqueLimits(const
Eigen::VectorXd &torqueLimits)

Set the torque limit.

Parameters:
-----------

torqueLimits:  The torque limit $ \\\\torque_{max} $ ";

%feature("docstring")  orcisir::TorqueLimitFunction::getTorqueLimits "const Eigen::VectorXd &
orcisir::TorqueLimitFunction::getTorqueLimits() const

Get the torque limit.

The torque limit $ \\\\torque_{max} $ ";


// File: classorcisir_1_1_variable_chi_function.xml
%feature("docstring") orcisir::VariableChiFunction "";

%feature("docstring")
orcisir::VariableChiFunction::VariableChiFunction "orcisir::VariableChiFunction::VariableChiFunction(orc::Variable &x,
int dimension) ";

%feature("docstring")
orcisir::VariableChiFunction::doUpdateInputSizeBegin "void
orcisir::VariableChiFunction::doUpdateInputSizeBegin() ";

%feature("docstring")
orcisir::VariableChiFunction::doUpdateInputSizeEnd "void
orcisir::VariableChiFunction::doUpdateInputSizeEnd() ";


// File: namespaceorc.xml


// File: namespaceorcisir.xml
%feature("docstring")  orcisir::getModelFromSharedLibrary "orc::Model* orcisir::getModelFromSharedLibrary(const std::string
&libPath, const std::string &createFunctionName, const std::string
&robotName) ";


// File: _contact_avoidance_constraint_8h.xml


// File: _i_s_i_r_constraint_8h.xml


// File: _joint_limit_constraint_8h.xml


// File: _torque_limit_constraint_8h.xml


// File: _i_s_i_r_feature_8h.xml


// File: _i_s_i_r_partial_state_8h.xml


// File: _i_s_i_r_controller_8h.xml


// File: _i_s_i_r_debug_8h.xml


// File: _model_8h.xml


// File: _performances_8h.xml


// File: _i_s_i_r_solver_8h.xml


// File: _one_level_solver_8h.xml


// File: _i_s_i_r_task_8h.xml


// File: _contact_avoidance_constraint_8cpp.xml


// File: _i_s_i_r_constraint_8cpp.xml


// File: _joint_limit_constraint_8cpp.xml


// File: _torque_limit_constraint_8cpp.xml


// File: _i_s_i_r_feature_8cpp.xml


// File: _i_s_i_r_partial_state_8cpp.xml


// File: _i_s_i_r_controller_8cpp.xml


// File: _i_s_i_r_debug_8cpp.xml


// File: _i_s_i_r_solver_8cpp.xml


// File: _one_level_solver_8cpp.xml


// File: _i_s_i_r_task_8cpp.xml


// File: group__constraint.xml


// File: group__feature.xml


// File: group__core.xml


// File: group__solver.xml


// File: group__task.xml


// File: todo.xml


// File: dir_3a0377ece039b18ab0bd8044527db54a.xml


// File: dir_012677b4836afffff7730d56ce0ef360.xml


// File: dir_3c7fb145e6bdd445ac799c48d65bede1.xml


// File: dir_9e3ca8e33ce48e6ffff082202df76aad.xml


// File: dir_3d67e2cbcfbfed8683a66faed34fac84.xml


// File: dir_76ca5284cc03fef12a8c7d640aa4bbf2.xml


// File: dir_0fd33cd84cb6322e464d0a56520d6032.xml


// File: dir_38315ef12da9ccdbc7b58364b41edb31.xml


// File: dir_76d8aae74bb2115dfe5b1fa222416e5c.xml


// File: dir_0752467cf7b9f1ad0895fe0358616fd6.xml


// File: dir_3e2dff282e45a899f334a09eac9bb1c5.xml


// File: dir_64e319e61d32cb94500207c4e20ed02f.xml


// File: dir_f81d3f1ba49a3898b83267a7597fece7.xml


// File: dir_7b629eeb491a08c0c975c5e4d1b02e13.xml


// File: dir_ef4c8f334dfd97d664e32a0d11b67d12.xml


// File: dir_1bb70fe7f43bb7cc9ad861a3f11da063.xml

