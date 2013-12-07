// isir_controller_xde_model.i - SWIG interface to get a orc::Model from a xde::gvm::extra::DynamicModel

%newobject getModelFromXDEDynamicModel;

%inline
%{

#include "gvm/DynamicModel.h"

class XDEDynamicModel: public orc::Model
{
public:
    XDEDynamicModel(xde::gvm::extra::DynamicModel& xdeModel)
        :orc::Model(xdeModel.getName(), xdeModel.nbDofs(), !xdeModel.hasFixedRoot())
        , _m(xdeModel)
        , _actuatedDofs(xdeModel.nbInternalDofs())
        , _linearTerms(xdeModel.nbDofs())
    {
        _actuatedDofs.setOnes();
        _linearTerms.setZero();
        
        _segmentName.resize(nbSegments());
        for (int i=0; i<nbSegments(); ++i)
        {
            _segmentName[i] = _m.getSegmentName(i); 
        }
    };

    virtual ~XDEDynamicModel() {};


    //================ General Methods =================//
    int                          nbSegments               () const {return _m.nbSegments();};
    const Eigen::VectorXd&       getActuatedDofs          () const {return _actuatedDofs;};
    const Eigen::VectorXd&       getJointLowerLimits      () const {return _m.getJointLowerLimits();};
    const Eigen::VectorXd&       getJointUpperLimits      () const {return _m.getJointUpperLimits();};
    const Eigen::VectorXd&       getJointPositions        () const {return _m.getJointPositions();};
    const Eigen::VectorXd&       getJointVelocities       () const {return _m.getJointVelocities();};
    const Eigen::Displacementd&  getFreeFlyerPosition     () const {return _m.getFreeFlyerPosition();};
    const Eigen::Twistd&         getFreeFlyerVelocity     () const {return _m.getFreeFlyerVelocity();};

    //================ Dynamic Methods =================//
    const Eigen::MatrixXd&       getInertiaMatrix         () const {return _m.getInertiaMatrix();};
    const Eigen::MatrixXd&       getInertiaMatrixInverse  () const {return _m.getInertiaMatrixInverse();};
    const Eigen::MatrixXd&       getDampingMatrix         () const {return _m.getDampingMatrix();};
    const Eigen::VectorXd&       getNonLinearTerms        () const {return _m.getNonLinearTerms();};
    const Eigen::VectorXd&       getLinearTerms           () const {return _linearTerms;}; // the _m.getLinearTerms() method throws an error.
    const Eigen::VectorXd&       getGravityTerms          () const {return _m.getGravityTerms();};

    //================== CoM Methods ===================//
    double                                         getMass                     () const {return _m.getMass();};
    const Eigen::Vector3d&                         getCoMPosition              () const {return _m.getCoMPosition();};
    const Eigen::Vector3d&                         getCoMVelocity              () const {return _m.getCoMVelocity();};
    const Eigen::Vector3d&                         getCoMJdotQdot              () const {return _m.getCoMJdotQdot();};
    const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobian              () const {return _m.getCoMJacobian();};
    const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobianDot           () const {throw std::runtime_error("XDEDynamicModel::getCoMJacobianDot not implemented.");};

    //================ Segments Methods ================//
    double                                         getSegmentMass              (int index) const {return _m.getSegmentMass(index);};
    const Eigen::Vector3d&                         getSegmentCoM               (int index) const {return _m.getSegmentCoM(index);};
    const Eigen::Matrix<double,6,6>&               getSegmentMassMatrix        (int index) const {return _m.getSegmentMassMatrix(index);};
    const Eigen::Vector3d&                         getSegmentMomentsOfInertia  (int index) const {return _m.getSegmentMomentsOfInertia(index);};
    const Eigen::Rotation3d&                       getSegmentInertiaAxes       (int index) const {return _m.getSegmentInertiaAxes(index);};
    const Eigen::Displacementd&                    getSegmentPosition          (int index) const {return _m.getSegmentPosition(index);};
    const Eigen::Twistd&                           getSegmentVelocity          (int index) const {return _m.getSegmentVelocity(index);};
    const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJacobian          (int index) const {return _m.getSegmentJacobian(index);};
    const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJdot              (int index) const {throw std::runtime_error("XDEDynamicModel::getSegmentJdot not implemented.");};
    const Eigen::Matrix<double,6,Eigen::Dynamic>&  getJointJacobian            (int index) const {return _m.getJointJacobian(index);};
    const Eigen::Twistd&                           getSegmentJdotQdot          (int index) const {return _m.getSegmentJdotQdot(index);};


protected:
    //=============== Set State Methods ================//
    void                doSetJointPositions     (const Eigen::VectorXd& q) {_m.setJointPositions(q);};
    void                doSetJointVelocities    (const Eigen::VectorXd& dq) {_m.setJointVelocities(dq);};
    void                doSetFreeFlyerPosition  (const Eigen::Displacementd& Hroot) {_m.setFreeFlyerPosition(Hroot);};
    void                doSetFreeFlyerVelocity  (const Eigen::Twistd& Troot) {_m.setFreeFlyerVelocity(Troot);};

    //=============== Index Name Methods ===============//
    int                 doGetSegmentIndex       (const std::string& name) const {return _m.getSegmentIndex(name);};
    const std::string&  doGetSegmentName        (int index) const {return _segmentName[index];};

    xde::gvm::extra::DynamicModel&  _m; // the inner model
    Eigen::VectorXd                 _actuatedDofs;
    Eigen::VectorXd                 _linearTerms;
    std::vector< std::string >      _segmentName;
};


XDEDynamicModel* getModelFromXDEDynamicModel(xde::gvm::extra::DynamicModel& xdeModel)
{
    return new XDEDynamicModel(xdeModel);
}

%}
