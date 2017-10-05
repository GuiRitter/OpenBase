#include <open_base_controller/custom_controller.h>

#define Ts 0.8
#define Xi 1.0
#define Wn (4.0/Ts/Xi)

namespace open_base_controller {

    CustomController::CustomController(void):
        q(0),dq(0),v(0),qr(0),dqr(0),ddqr(0),torque(0),fext(0) {
    }

    CustomController::~CustomController(void) {
        sub_command_.shutdown();
    }

    bool CustomController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {
        node_=n;
        robot_=robot;

        XmlRpc::XmlRpcValue joint_names;
        if (!node_.getParam("joints",joint_names)) {
            ROS_ERROR("No 'joints' in controller. (namespace: %s)", node_.getNamespace().c_str());
            return false;
        }

        if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("'joints' is not a struct. (namespace: %s)", node_.getNamespace().c_str());
            return false;
        }

        for (int i=0; i < joint_names.size(); i++) {
            XmlRpc::XmlRpcValue &name_value = joint_names[i];
            if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("joints are not strings. (namespace: %s)", node_.getNamespace().c_str());
                return false;
            }
            ROS_WARN("%d\t%s", i, ((std::string) name_value).c_str());
            hardware_interface::JointHandle j = robot->getHandle((std::string) name_value);
            jointVector.push_back(j);
        }
        sub_command_ = node_.subscribe("command", 1, &CustomController::commandCallBack, this);

        std::string robot_desc_string;
        if (!node_.getParam("/open_base/robot_description", robot_desc_string)) {
            ROS_ERROR("Could not find 'robot_description'.");
            return false;
        }

        KDL::Tree tree;
        if (!kdl_parser::treeFromString(robot_desc_string, tree)) {
            ROS_ERROR("Failed to construct KDL tree.");
            return false;
        }

        /*KDL::Chain chain;
        if (!tree.getChain("open_base_origin", "open_base_tool_plate",chain)) {
            ROS_ERROR("Failed to get chain from KDL tree.");
            return false;
        }*/

        KDL::Vector g;
        node_.param("/gazebo/gravity_x", g[0],  0.0);
        node_.param("/gazebo/gravity_y", g[1],  0.0);
        node_.param("/gazebo/gravity_z", g[2], -9.8);

	/*if ((idsolver=new KDL::ChainIdSolver_RNE(chain,g)) == NULL) {
	    ROS_ERROR("Failed to create ChainIDSolver_RNE.");
	    return false;
	}

	q.resize(chain.getNrOfJoints());
	dq.resize(chain.getNrOfJoints());
	v.resize(chain.getNrOfJoints());
	qr.resize(chain.getNrOfJoints());
	dqr.resize(chain.getNrOfJoints());
	ddqr.resize(chain.getNrOfJoints());
	torque.resize(chain.getNrOfJoints());

	fext.resize(chain.getNrOfSegments());

	Kp.resize(chain.getNrOfJoints(),chain.getNrOfJoints());
	Kd.resize(chain.getNrOfJoints(),chain.getNrOfJoints());*/

        return true;
    }

    void CustomController::starting(const ros::Time& time) {
        /*Kp.setZero();
        Kd.setZero();
        for(unsigned int i=0;i < jointVector.size();i++) {
            Kp(i,i)=Wn*Wn;
            Kd(i,i)=2.0*Xi*Wn;
            q(i)=jointVector[i].getPosition();
            dq(i)=jointVector[i].getVelocity();
        }
        qr=q;
        dqr=dq;
        SetToZero(ddqr);*/

        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            ROS_WARN("Failed to set real-time scheduler.");
            return;
        }
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            ROS_WARN("Failed to lock memory.");
        }
    }

    void CustomController::update(const ros::Time& time, const ros::Duration& duration) {
        /*
        jointVector[0].setCommand(0.02);
        jointVector[1].setCommand(0.02);
        jointVector[2].setCommand(0.02);
        max = std::max(jointVector[0].getVelocity(), max);
        max = std::max(jointVector[1].getVelocity(), max);
        max = std::max(jointVector[2].getVelocity(), max);
        ROS_WARN("%f", max);
        */
        //ROS_WARN("%lf", time.toSec());
        //ROS_WARN("%lf\t%lf\t%lf", jointVector[0].getVelocity(), jointVector[1].getVelocity(), jointVector[2].getVelocity());
        /*
        if ((j > 0) && (j < 1001)) {
            ROS_WARN("%lf\t%lf\t%lf\t%lf\t%lf", time.toSec(), u[j - 1], jointVector[0].getVelocity(), jointVector[1].getVelocity(), jointVector[2].getVelocity());
        }
        /**/
        if ((j > -1) && (j < 1000)) {
            ROS_WARN("%lf\t%lf\t%lf\t%lf\t%lf", time.toSec(), u[j], jointVector[0].getVelocity(), jointVector[1].getVelocity(), jointVector[2].getVelocity());
            jointVector[0].setCommand(u[j]);
            jointVector[1].setCommand(u[j]);
            jointVector[2].setCommand(u[j]);
        }
        j++;
        /**/
        /*for (unsigned int i=0;i < jointVector.size();i++) {
            q(i)=jointVector[i].getPosition();
            dq(i)=jointVector[i].getVelocity();
        }
        for (unsigned int i=0;i < fext.size();i++) {
            fext[i].Zero();
        }

        v.data = ddqr.data + Kp * (qr.data - q.data) + Kd * (dqr.data - dq.data);
        if (idsolver->CartToJnt(q, dq, v, fext, torque) < 0) {
            ROS_ERROR("KDL inverse dynamics solver failed.");
        }

        for (unsigned int i=0; i < jointVector.size(); i++) {
            jointVector[i].setCommand(torque(i));
        }*/
    }

    void CustomController::commandCallBack(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &referencePoint) {
        /*for (unsigned int i=0 ;i < qr.rows(); i++) {
            qr(i) = referencePoint->positions[i];
            dqr(i) = referencePoint->velocities[i];
            ddqr(i) = referencePoint->accelerations[i];
        }*/
    }
}

PLUGINLIB_EXPORT_CLASS(open_base_controller::CustomController, controller_interface::ControllerBase)
