
/*
 * \file  gazebo_sphero_controller.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot. The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * A modification from the original Differential drive of Gazebo
 * To make the Sphero simulated robot move using any keyboard teleop
 * \author   Ricardo Tellez <rtellez@theconstructsim.com>
 * \date 11th of Aug 2016
 *
 * $ Id: 08/11/2016 20:05:40 PM ouroboros $
 */


#include <algorithm>
#include <assert.h>

#include <sphero_gazebo/gazebo_sphero_controller.h>

#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{

enum {
    RIGHT,
    LEFT,
};

GazeboSpheroController::GazeboSpheroController() {}

// Destructor
GazeboSpheroController::~GazeboSpheroController() {}

// Load the controller
void GazeboSpheroController::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );

    gazebo_ros_->getParameter<double> ( wheel_separation_, "wheelSeparation", 0.34 );
    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.15 );
    gazebo_ros_->getParameter<double> ( wheel_accel, "wheelAcceleration", 0.0 );
    gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );


    joints_.resize ( 2 );
    joints_[LEFT] = gazebo_ros_->getJoint ( parent, "leftJoint", "left_joint" );
    joints_[RIGHT] = gazebo_ros_->getJoint ( parent, "rightJoint", "right_joint" );
    joints_[LEFT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT]->SetParam ( "fmax", 0, wheel_torque );


    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN("GazeboSpheroController Plugin (ns = %s) missing <publishTf>, defaults to %d",
          this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
    last_update_time_ = parent->GetWorld()->SimTime();

    // Initialize velocity stuff
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;


    if (this->publishWheelJointState_)
    {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO("%s: Advertise joint_states!", gazebo_ros_->info());
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboSpheroController::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      ROS_INFO("%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str());
    }

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboSpheroController::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboSpheroController::UpdateChild, this ) );

}

void GazeboSpheroController::Reset()
{
  last_update_time_ = parent->GetWorld()->SimTime();
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;
  joints_[LEFT]->SetParam ( "fmax", 0, wheel_torque );
  joints_[RIGHT]->SetParam ( "fmax", 0, wheel_torque );
}

void GazeboSpheroController::publishWheelJointState()
{
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );

    for ( int i = 0; i < 2; i++ ) {
        physics::JointPtr joint = joints_[i];
        ::ignition::math::Angle angle = joint->Position ( 0 );
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = angle.Radian () ;
    }
    joint_state_publisher_.publish ( joint_state_ );
}

void GazeboSpheroController::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < 2; i++ ) {

        std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
        std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());

        ::ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose();

        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
}

// Update the controller
void GazeboSpheroController::UpdateChild()
{

    /* force reset SetMaxForce since Joint::Reset reset MaxForce to zero at
       https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
       (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
       and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than GazeboSpheroController::Reset
       (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
    */
    for ( int i = 0; i < 2; i++ ) {
      if ( fabs(wheel_torque - joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
      	joints_[i]->SetParam ( "fmax", 0, wheel_torque );
      }
    }


    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
    common::Time current_time = parent->GetWorld()->SimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();

        double current_speed[2];

        current_speed[LEFT] = joints_[LEFT]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[RIGHT] = joints_[RIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );

        if ( wheel_accel == 0 ||
                ( fabs ( wheel_speed_[LEFT] - current_speed[LEFT] ) < 0.01 ) ||
                ( fabs ( wheel_speed_[RIGHT] - current_speed[RIGHT] ) < 0.01 ) ) {
            //if max_accel == 0, or target speed is reached
            joints_[LEFT]->SetVelocity ( 0, wheel_speed_[LEFT]/ ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT]->SetVelocity ( 0, wheel_speed_[RIGHT]/ ( wheel_diameter_ / 2.0 ) );
        } else {
            if ( wheel_speed_[LEFT]>=current_speed[LEFT] )
                wheel_speed_instr_[LEFT]+=fmin ( wheel_speed_[LEFT]-current_speed[LEFT],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[LEFT]+=fmax ( wheel_speed_[LEFT]-current_speed[LEFT], -wheel_accel * seconds_since_last_update );

            if ( wheel_speed_[RIGHT]>current_speed[RIGHT] )
                wheel_speed_instr_[RIGHT]+=fmin ( wheel_speed_[RIGHT]-current_speed[RIGHT], wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[RIGHT]+=fmax ( wheel_speed_[RIGHT]-current_speed[RIGHT], -wheel_accel * seconds_since_last_update );

            // ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
            // ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);

            joints_[LEFT]->SetVelocity ( 0,wheel_speed_instr_[LEFT] / ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT]->SetVelocity ( 0,wheel_speed_instr_[RIGHT] / ( wheel_diameter_ / 2.0 ) );
        }

        last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void GazeboSpheroController::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboSpheroController::getWheelVelocities()
{
    boost::mutex::scoped_lock scoped_lock ( lock );

    double vr = x_;
    double va = rot_;

    wheel_speed_[LEFT] = vr; // + va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT] =   - va * wheel_separation_ / 2.0;
}

void GazeboSpheroController::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;

}

void GazeboSpheroController::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboSpheroController::UpdateOdometryEncoder()
{
    double vl = joints_[LEFT]->GetVelocity ( 0 );
    double vr = joints_[RIGHT]->GetVelocity ( 0 );
    common::Time current_time = parent->GetWorld()->SimTime();
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double b = wheel_separation_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double theta = ( sl - sr ) /b;


    double dx = ( sl + sr ) /2.0 * cos ( pose_encoder_.theta + ( sl - sr ) / ( 2.0*b ) );
    double dy = ( sl + sr ) /2.0 * sin ( pose_encoder_.theta + ( sl - sr ) / ( 2.0*b ) );
    double dtheta = ( sl - sr ) /b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dx/seconds_since_last_update;
    odom_.twist.twist.linear.y = dy/seconds_since_last_update;
}

void GazeboSpheroController::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data form gazebo world
        ::ignition::math::Pose3d pose = parent->WorldPose();
        qt = tf::Quaternion ( 0, 0, 0, 1 );
        vt = tf::Vector3 ( pose.Pos.X(), pose.Pos.Y(), pose.Pos.Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        ::ignition::math::Vector3 linear;
        linear = parent->GetWorldLinearVel();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Z();

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    tf::Transform base_footprint_to_odom ( qt, vt );
    transform_broadcaster_->sendTransform (
        tf::StampedTransform ( base_footprint_to_odom, current_time,
                               odom_frame, base_footprint_frame ) );


    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboSpheroController )
}

