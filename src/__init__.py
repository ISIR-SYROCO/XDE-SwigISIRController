
# For the model part
from swig_isir_controller import getModelFromSharedLibrary

# For the solver part
from swig_isir_controller import OneLevelSolverWithQuadProg, OneLevelSolverWithQLD

#For the Feature part
from swig_isir_controller import FullModelState, FullTargetState, FullStateFeature, \
    PartialModelState, PartialTargetState, PartialStateFeature, \
    ControlFrame, TargetFrame, SegmentFrame, CoMFrame, \
    PositionFeature, PointContactFeature, OrientationFeature, DisplacementFeature

from swig_isir_controller import FullState

FULL_STATE = FullState.FULL_STATE
FREE_FLYER = FullState.FREE_FLYER
INTERNAL   = FullState.INTERNAL

from swig_isir_controller import X, Y, XY, Z, XZ, YZ, XYZ

# For the task part
from swig_isir_controller import ISIRTask

UNKNOWNTASK      = ISIRTask.UNKNOWNTASK
ACCELERATIONTASK = ISIRTask.ACCELERATIONTASK
TORQUETASK       = ISIRTask.TORQUETASK
FORCETASK        = ISIRTask.FORCETASK

# For the constraint part
from swig_isir_controller import TorqueLimitConstraint, JointLimitConstraint, ContactAvoidanceConstraint

# For the controller part
from swig_isir_controller import ISIRController


################################################################################

# The following are available only if their corresponding libraries were found
# For the model part (if it has been compiled)
try:
    from swig_isir_controller import getModelFromXDEDynamicModel
except:
    getModelFromXDEDynamicModel = None

# For the orocos part (if it has been compiled)
try:
    from swig_isir_controller import ProxyModel       , RemoteModel       , \
                                     ProxySegmentFrame, RemoteSegmentFrame, \
                                     ProxyCoMFrame    , RemoteCoMFrame    , \
                                     ProxyTargetFrame , RemoteTargetFrame
except:
    ProxyModel        = RemoteModel        = None
    ProxySegmentFrame = RemoteSegmentFrame = None
    ProxyCoMFrame     = RemoteCoMFrame     = None
    ProxyTargetFrame  = RemoteTargetFrame  = None

try:
    from isir_task_manager import TaskXMLParser
except:
    print "ISIRTaskManager not installed"
    TaskXMLParser = None






