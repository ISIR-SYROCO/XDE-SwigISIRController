// isir_controller_orocos_components.i - SWIG interface to get communication between orocos component that make proxies of isir_controller classes

%inline
%{

#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>


////////////////////////////////////////////////////////////////////////////////
//
// Remote/Proxy CLASS
//
////////////////////////////////////////////////////////////////////////////////

class _RemoteComponent
{
public:
    _RemoteComponent(const std::string& name)
        : innerTaskContext(name)
        {}

    RTT::TaskContext* getTaskContext() {return &innerTaskContext;};

protected:
    RTT::TaskContext innerTaskContext;
};

////////////////////////////////////////////////////////////////////////////////
//
// Remote/Proxy CLASS FOR orc::FullModelState & orc::PartialModelState
//
////////////////////////////////////////////////////////////////////////////////

// CANNOT CREATE PROXY AND REMOTE FOR FULLSTATE & PARTIALSTATE CLASS!!!

// Their getJacobian methods cannot be virtualized, so they cannot receive
// the data from a remote class.
// Nevertheless, matrices are constant and easy to write. So that can be done
// without a remote/proxy class as done here.


////////////////////////////////////////////////////////////////////////////////
//
// Remote/Proxy CLASS FOR orc::ControlFrame
//
////////////////////////////////////////////////////////////////////////////////
class _RemoteControlFrame: public _RemoteComponent
{
public:
    _RemoteControlFrame(const std::string& name, orc::ControlFrame& CF)
        : _RemoteComponent(name)
        , innerControlFrame(CF)
    {
        innerTaskContext.addOperation("getPosition"                , &_RemoteControlFrame::getPosition                , this,  RTT::OwnThread);
        innerTaskContext.addOperation("getVelocity"                , &_RemoteControlFrame::getVelocity                , this,  RTT::OwnThread);
        innerTaskContext.addOperation("getAcceleration"            , &_RemoteControlFrame::getAcceleration            , this,  RTT::OwnThread);
        innerTaskContext.addOperation("getWrench"                  , &_RemoteControlFrame::getWrench                  , this,  RTT::OwnThread);
        innerTaskContext.addOperation("getJacobian"                , &_RemoteControlFrame::getJacobian                , this,  RTT::OwnThread);
        innerTaskContext.addOperation("dependsOnModelConfiguration", &_RemoteControlFrame::dependsOnModelConfiguration, this,  RTT::OwnThread);
        //innerTaskContext.addOperation("getModel"                   , &RemoteControlFrame::getModel, this,  RTT::OwnThread); //NO NO!! Cannot be convert in operation
    }

    Eigen::Displacementd                    getPosition()                   const {return innerControlFrame.getPosition();};
    Eigen::Twistd                           getVelocity()                   const {return innerControlFrame.getVelocity();};
    Eigen::Twistd                           getAcceleration()               const {return innerControlFrame.getAcceleration();};
    Eigen::Wrenchd                          getWrench()                     const {return innerControlFrame.getWrench();};
    Eigen::Matrix<double,6,Eigen::Dynamic>  getJacobian()                   const {return innerControlFrame.getJacobian();};
    bool                                    dependsOnModelConfiguration()   const {return innerControlFrame.dependsOnModelConfiguration();};
    const Model&                            getModel()                      const {return innerControlFrame.getModel();};

private:
    orc::ControlFrame& innerControlFrame;
};


class _ProxyControlFrame: public orc::ControlFrame
{
public:
    _ProxyControlFrame(const std::string& name, RTT::TaskContext& RTC)
        : orc::ControlFrame(name)
        , remoteTaskContext(RTC)
    {
        proxy_getPosition                 = remoteTaskContext.getOperation("getPosition");
        proxy_getVelocity                 = remoteTaskContext.getOperation("getVelocity");
        proxy_getAcceleration             = remoteTaskContext.getOperation("getAcceleration");
        proxy_getWrench                   = remoteTaskContext.getOperation("getWrench");
        proxy_getJacobian                 = remoteTaskContext.getOperation("getJacobian");
        proxy_dependsOnModelConfiguration = remoteTaskContext.getOperation("dependsOnModelConfiguration");
    }

    Eigen::Displacementd                    getPosition()                   const {return const_cast< _ProxyControlFrame* >(this)->proxy_getPosition();};
    Eigen::Twistd                           getVelocity()                   const {return const_cast< _ProxyControlFrame* >(this)->proxy_getVelocity();};
    Eigen::Twistd                           getAcceleration()               const {return const_cast< _ProxyControlFrame* >(this)->proxy_getAcceleration();};
    Eigen::Wrenchd                          getWrench()                     const {return const_cast< _ProxyControlFrame* >(this)->proxy_getWrench();};
    Eigen::Matrix<double,6,Eigen::Dynamic>  getJacobian()                   const {return const_cast< _ProxyControlFrame* >(this)->proxy_getJacobian();};
    bool                                    dependsOnModelConfiguration()   const {return const_cast< _ProxyControlFrame* >(this)->proxy_dependsOnModelConfiguration();};
    const Model&                            getModel()                      const {throw std::runtime_error("[InputControlFrame::getModel] should not be called.");};

protected:
    RTT::TaskContext&  remoteTaskContext;

    RTT::OperationCaller< Eigen::Displacementd (void) >                     proxy_getPosition;
    RTT::OperationCaller< Eigen::Twistd (void) >                            proxy_getVelocity;
    RTT::OperationCaller< Eigen::Twistd (void) >                            proxy_getAcceleration;
    RTT::OperationCaller< Eigen::Wrenchd (void) >                           proxy_getWrench;
    RTT::OperationCaller< Eigen::Matrix<double,6,Eigen::Dynamic> (void) >   proxy_getJacobian;
    RTT::OperationCaller< bool (void) >                                     proxy_dependsOnModelConfiguration;
};




////////////////////////////////////////////////////////////////////////////////
//
// Remote/Proxy CLASS FOR orc::SegmentFrame
//
////////////////////////////////////////////////////////////////////////////////
class RemoteSegmentFrame: public orc::SegmentFrame, public _RemoteControlFrame
{
public:
    RemoteSegmentFrame(const std::string& name, const Model& model, const std::string& segname)
        : orc::SegmentFrame(name, model, segname)
        , _RemoteControlFrame(name, static_cast<orc::SegmentFrame&>(*this))
    {
        innerTaskContext.addOperation("getSegmentIndex", &RemoteSegmentFrame::getSegmentIndex, this,  RTT::OwnThread);
    }

    RemoteSegmentFrame(const std::string& name, const Model& model, const std::string& segname, const Eigen::Displacementd& H_local)
        : orc::SegmentFrame(name, model, segname, H_local)
        , _RemoteControlFrame(name, static_cast<orc::SegmentFrame&>(*this))
    {
        innerTaskContext.addOperation("getSegmentIndex", &RemoteSegmentFrame::getSegmentIndex, this,  RTT::OwnThread);
    }

    RemoteSegmentFrame(const std::string& name, const Model& model, int segmentId)
        : orc::SegmentFrame(name, model, segmentId)
        , _RemoteControlFrame(name, static_cast<orc::SegmentFrame&>(*this))
    {
        innerTaskContext.addOperation("getSegmentIndex", &RemoteSegmentFrame::getSegmentIndex, this,  RTT::OwnThread);
    }

    RemoteSegmentFrame(const std::string& name, const Model& model, int segmentId, const Eigen::Displacementd& H_local)
        : orc::SegmentFrame(name, model, segmentId, H_local)
        , _RemoteControlFrame(name, static_cast<orc::SegmentFrame&>(*this))
    {
        innerTaskContext.addOperation("getSegmentIndex", &RemoteSegmentFrame::getSegmentIndex, this,  RTT::OwnThread);
    }
};


class ProxySegmentFrame: public _ProxyControlFrame
{
public:
    ProxySegmentFrame(const std::string& name, RTT::TaskContext& RTC)
        : _ProxyControlFrame(name, RTC)
    {
        proxy_getSegmentIndex = remoteTaskContext.getOperation("getSegmentIndex");
    }

    int getSegmentIndex() const {return const_cast< ProxySegmentFrame* >(this)->proxy_getSegmentIndex();};

protected:
    RTT::OperationCaller< int (void) >  proxy_getSegmentIndex;
};



////////////////////////////////////////////////////////////////////////////////
//
// Remote/Proxy CLASS FOR orc::CoMFrame
//
////////////////////////////////////////////////////////////////////////////////
class RemoteCoMFrame: public orc::CoMFrame, public _RemoteControlFrame
{
public:
    RemoteCoMFrame(const std::string& name, const orc::Model& model)
        : orc::CoMFrame(name, model)
        , _RemoteControlFrame(name, static_cast<orc::CoMFrame&>(*this))
    {}
};

class ProxyCoMFrame: public _ProxyControlFrame
{
public:
    ProxyCoMFrame(const std::string& name, RTT::TaskContext& RTC)
        : _ProxyControlFrame(name, RTC)
    {}
};


////////////////////////////////////////////////////////////////////////////////
//
// Remote/Proxy CLASS FOR orc::TargetFrame
//
////////////////////////////////////////////////////////////////////////////////
class RemoteTargetFrame: public orc::TargetFrame, public _RemoteControlFrame
{
public:
    RemoteTargetFrame(const std::string& name, const orc::Model& model)
        : orc::TargetFrame(name, model)
        , _RemoteControlFrame(name, static_cast<orc::TargetFrame&>(*this))
    {
        innerTaskContext.addOperation("setPosition"    , &RemoteTargetFrame::setPosition    , this,  RTT::OwnThread);
        innerTaskContext.addOperation("setVelocity"    , &RemoteTargetFrame::setVelocity    , this,  RTT::OwnThread);
        innerTaskContext.addOperation("setAcceleration", &RemoteTargetFrame::setAcceleration, this,  RTT::OwnThread);
        innerTaskContext.addOperation("setWrench"      , &RemoteTargetFrame::setWrench      , this,  RTT::OwnThread);
    }
};


class ProxyTargetFrame: public _ProxyControlFrame
{
public:
    ProxyTargetFrame(const std::string& name, RTT::TaskContext& RTC)
        : _ProxyControlFrame(name, RTC)
    {
        proxy_setPosition     = remoteTaskContext.getOperation("setPosition");
        proxy_setVelocity     = remoteTaskContext.getOperation("setVelocity");
        proxy_setAcceleration = remoteTaskContext.getOperation("setAcceleration");
        proxy_setWrench       = remoteTaskContext.getOperation("setWrench");
    }
    void setPosition(const Eigen::Displacementd& H)     {proxy_setPosition(H);};
    void setVelocity(const Eigen::Twistd& T)            {proxy_setVelocity(T);};
    void setAcceleration(const Eigen::Twistd& gamma)    {proxy_setAcceleration(gamma);};
    void setWrench(const Eigen::Wrenchd& W)             {proxy_setWrench(W);};

protected:
    RTT::OperationCaller< void (const Eigen::Displacementd&) >      proxy_setPosition;
    RTT::OperationCaller< void (const Eigen::Twistd&) >             proxy_setVelocity;
    RTT::OperationCaller< void (const Eigen::Twistd&) >             proxy_setAcceleration;
    RTT::OperationCaller< void (const Eigen::Wrenchd&) >            proxy_setWrench;
};






////////////////////////////////////////////////////////////////////////////////
//
// Remote/Proxy CLASS FOR orc::Model
//
////////////////////////////////////////////////////////////////////////////////
class RemoteModel
{
public:
    RemoteModel(const std::string& taskContextName, orc::Model& model)
        : innerTaskContext(taskContextName)
        , innerModel(model)
    {
        innerTaskContext.addOperation("updateModel", &RemoteModel::updateModel, this,  RTT::OwnThread);
    }

    virtual void updateModel(const Eigen::Displacementd& H_root, const Eigen::VectorXd& q, const Eigen::Twistd& T_root, const Eigen::VectorXd& q_dot)
    {
        if (innerModel.hasFixedRoot())
        {
            innerModel.setState(q, q_dot);
            innerModel.setFreeFlyerPosition(H_root);
        }
        else
        {
        innerModel.setState(H_root, q, T_root, q_dot);
        }
    }

    RTT::TaskContext* getTaskContext() {return &innerTaskContext;};

private:
    RTT::TaskContext    innerTaskContext;
    orc::Model&         innerModel;
};

class ProxyModel
{
public:
    ProxyModel(orc::Model& model, RTT::TaskContext& RTC)
        : remoteTaskContext(RTC)
        , innerModel(model)
    {
        proxy_updateModel = remoteTaskContext.getOperation("updateModel");
    }

    virtual void updateModel()
    {
        Eigen::Displacementd H_root = innerModel.getFreeFlyerPosition();
        Eigen::VectorXd      q      = innerModel.getJointPositions();
        Eigen::Twistd        T_root = innerModel.hasFixedRoot() ? Eigen::Twistd() : innerModel.getFreeFlyerVelocity();
        Eigen::VectorXd      q_dot  = innerModel.getJointVelocities();

        proxy_updateModel(H_root, q, T_root, q_dot);
    };

private:
    RTT::TaskContext&  remoteTaskContext;
    orc::Model&         innerModel;

    RTT::OperationCaller< void (const Eigen::Displacementd&, const Eigen::VectorXd&, const Eigen::Twistd&, const Eigen::VectorXd&) > proxy_updateModel;
};

%} // end of inline



