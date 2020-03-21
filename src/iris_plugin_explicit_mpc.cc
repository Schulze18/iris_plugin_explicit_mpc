// This plugin is based on the tutorial to control a Velodyne sensor

#ifndef _IRIS_PLUGIN_EXPLICIT_MPC_HH_
#define _IRIS_PLUGIN_EXPLICIT_MPC_HH_

// Include default libraries for Gazebo and ROS interface
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <thread>
#include <iostream>
#include <time.h>
#include <sys/time.h>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ignition/math/Pose3.hh>
#include <gazebo/common/SystemPaths.hh>
// Include Msgs Types
#include <sensor_msgs/Imu.h> 
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
// Basic libraries for C and C++
#include <stddef.h>
#include <stdio.h>
// Includes for explicit MPC BST
#include <explicit_mpc_bst.h>
#include "explicit_mpc_bst.cpp"
#include <vector>
#include <math.h>  


namespace gazebo
{
  // A plugin to control an Iris 3DR quadrotor.
  class IrisPluginExplicitMPC : public ModelPlugin
  {
    // Constructor
    public: IrisPluginExplicitMPC() {}

    // The load function is called by Gazebo when the plugin is inserted into simulation
    // \param[in] _model A pointer to the model that this plugin is attached to.
    // \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

   	// Just output a message for now
    std::cerr << "\nThe iris plugin is attach to model" << _model->GetName() << "\n";

	//Load Regions and BST file
	get_bst_from_file(&(this->bst_nodes), this->filename_bst);
	get_regions_from_file(&(this->regions), this->filename_regions);
	get_ineq_set_from_file(&(this->ineq_set), this->filename_ineq_set);
	get_control_param_file(&(this->control_param), this->filename_control_param);

	//std::cout << "test bst: " << (this->bst_nodes)[0].left << "\n";
	//std::cout << "test regions: " << this->regions[0].Kx[0] << "\n";

	// Store the model pointer for convenience.
	this->model = _model;	
	
	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
	{
	  int argc = 0;
	  char **argv = NULL;
	  ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

	//Create and subscribe to a topic with Quaternion type
	/*ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Point>(
	"/" + this->model->GetName() + "/iris_ref",1,
	boost::bind(&IrisPluginExplicitMPC::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
	*/

	//Create and subscribe to a topic with Pose type	
	ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
	"/" + this->model->GetName() + "/iris_ref",1,
	boost::bind(&IrisPluginExplicitMPC::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);

	// Store the subscriber for convenience.
	this->rosSub = this->rosNode->subscribe(so);

	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&IrisPluginExplicitMPC::QueueThread, this));

	// Create a topic to publish iris state
	this->state_pub = this->rosNode->advertise<sensor_msgs::Imu>("iris_state", 100);
	// Create a topic to publish the rotors velocities 
	this->vel_pub = this->rosNode->advertise<geometry_msgs::Quaternion>("vel_cmd", 100);
	
	// Configure Timer and callback function
	this->pubTimer = this->rosNode->createTimer(ros::Duration(0.01), &IrisPluginExplicitMPC::control_callback,this);
	
	// listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
	   event::Events::ConnectWorldUpdateBegin ( boost::bind ( &IrisPluginExplicitMPC::UpdateVelocity, this ) );

	/*
	//TOPIC
  	if (_sdf->HasElement("topicState"))
  	{
		//std::cerr << std::string(this->model->GetFilename()) << "\n";
		std::cout << _sdf->Get<std::string>("topicState") << "\n";
		std::cout << _sdf->Get<std::string>("topicCommand") << "\n";
		std::cout << _sdf->Get<std::string>("topicReference") << "\n";
		std::cout << _sdf->Get<std::string>("filenameBST") << "\n";
		std::cout << _sdf->Get<std::string>("filenameRegions") << "\n";
		std::cout << _sdf->Get<std::string>("filenameIneq") << "\n";
		std::cout << _sdf->Get<std::string>("filenameControlParam") << "\n";

		this->rosNode->getParam("/explicit_plugin_path", this->plugin_pkg_path);
		std::cout << this->plugin_pkg_path << "\n\n";

	}*/


	if (this->rosNode->hasParam("/explicit_plugin_path")){
		this->rosNode->getParam("/explicit_plugin_path", this->plugin_pkg_path);
	}
	else{
		ROS_INFO("No param named 'explicit_plugin_path'");
	}
	std::string temp_file_BST;
	std::string temp_file_regions;
	std::string temp_file_ineq;
	std::string temp_file_param;

	if (_sdf->HasElement("topicState")) std::string temp_topic_state = _sdf->Get<std::string>("topicState");
  	else std::string temp_topic_state = "iris_state";

	if (_sdf->HasElement("topicCommand")) std::string temp_topic_command = _sdf->Get<std::string>("topicCommand");
  	else std::string temp_topic_state = "vel_cmd";

	if (_sdf->HasElement("topicReference")) std::string temp_topic_reference = _sdf->Get<std::string>("topicReference");
  	else std::string temp_topic_state = "iris_ref";

	if (_sdf->HasElement("filenameBST")) temp_file_BST = _sdf->Get<std::string>("filenameBST");
  	else temp_file_BST = "/control_files/vt1_20_2_u3_linux/output_bst_20_2_u3_vt1.txt";

	if (_sdf->HasElement("filenameRegions")) temp_file_regions = _sdf->Get<std::string>("filenameRegions");
  	else temp_file_regions = "/control_files/vt1_20_2_u3_linux/output_regions_20_2_u3_vt1.txt";

	if (_sdf->HasElement("filenameIneq")) temp_file_ineq = _sdf->Get<std::string>("filenameIneq");
  	else  temp_file_ineq = "/control_files/vt1_20_2_u3_linux/output_ineq_20_2_u3_vt1.txt";

	if (_sdf->HasElement("filenameControlParam")) temp_file_param = _sdf->Get<std::string>("filenameControlParam");
  	else temp_file_param = "/control_files/vt1_20_2_u3_linux/output_control_param_20_2_u3_vt1.txt";

	std::string string_temp1;
	std::string string_temp2;
	std::string string_temp3;
	std::string string_temp4;
	/*string_temp = this->plugin_pkg_path + _sdf->Get<std::string>("filenameBST");
	//std::cout << "string temp:"  <<"\n" << string_temp << "\n";
	
	this->filename_control_test = string_temp.c_str();
	
	std::cout << "const char temp:"  << "\n" << this->filename_control_test << "\n";
	*/
	//string_temp1 = this->plugin_pkg_path + _sdf->Get<std::string>("filenameBST");
	string_temp1 = this->plugin_pkg_path + temp_file_BST;
	this->filename_bst = string_temp1.c_str();

	//string_temp2 = this->plugin_pkg_path + _sdf->Get<std::string>("filenameRegions");
	string_temp2 = this->plugin_pkg_path + temp_file_regions;
	this->filename_regions = string_temp2.c_str();

	//string_temp3 = this->plugin_pkg_path + _sdf->Get<std::string>("filenameIneq");
	string_temp3 = this->plugin_pkg_path + temp_file_ineq;
	this->filename_ineq_set = string_temp3.c_str();

	//string_temp4 = this->plugin_pkg_path + _sdf->Get<std::string>("filenameControlParam");
	string_temp4 = this->plugin_pkg_path + temp_file_param;
	this->filename_control_param = string_temp4.c_str();

	std::cout << "TEST Files\n";
	std::cout << this->filename_bst << "\n";
	std::cout << this->filename_regions << "\n";
	std::cout << this->filename_ineq_set << "\n";
	std::cout << this->filename_control_param << "\n";


	//Debug prints
	//this->model->GetLinks()
	//std::cout << this->model->GetLink("rotor_0") << "\n";

	}

    // Update the velocity applied to the Rotors
	public: void UpdateVelocity()
    {
    // Store the value of some aerodynamic constant
	double k = 0.000015674;
	double b = 0.000000114;	

	// vel is an array that get the rotors velocities calculated by the MPC
	double vel[4] = {this->iris_rotor_vel[0], this->iris_rotor_vel[1], this->iris_rotor_vel[2], this->iris_rotor_vel[3]};
	//vel[0] = 490; vel[1] = 490; vel[2] = 490; vel[3] = 490;
	
	// Create 
	math::Vector3 thrust_force;

	//rotor_0 - Link 6
	//rotor_1 - Link 5
	//rotor_2 - Link 4
	//rotor_3 - Link 3

	//Force Application
	//thrust_force.Set(0,0,k*vel[0]*vel[0]);		
	thrust_force.Set(0,0,this->iris_KT*vel[0]*vel[0]);
	this->model->GetLinks()[3]->AddRelativeForce(thrust_force);

	//thrust_force.Set(0,0,k*vel[1]*vel[1]);
	thrust_force.Set(0,0,this->iris_KT*vel[1]*vel[1]);	
	this->model->GetLinks()[4]->AddRelativeForce(thrust_force);

	//thrust_force.Set(0,0,k*vel[2]*vel[2]);	
	thrust_force.Set(0,0,this->iris_KT*vel[2]*vel[2]);	
	this->model->GetLinks()[5]->AddRelativeForce(thrust_force);

	//thrust_force.Set(0,0,k*vel[3]*vel[3]);
	thrust_force.Set(0,0,this->iris_KT*vel[3]*vel[3]);	
	this->model->GetLinks()[6]->AddRelativeForce(thrust_force);

	//Torque Application 
	math::Vector3 angular_torque;
	angular_torque.Set(0,0,-this->iris_KD*vel[0]*vel[0]);
	this->model->GetLinks()[3]->AddRelativeTorque(angular_torque);
	
	angular_torque.Set(0,0,-this->iris_KD*vel[1]*vel[1]);
	this->model->GetLinks()[4]->AddRelativeTorque(angular_torque);

	angular_torque.Set(0,0,this->iris_KD*vel[2]*vel[2]);
	this->model->GetLinks()[5]->AddRelativeTorque(angular_torque);

	angular_torque.Set(0,0,this->iris_KD*vel[3]*vel[3]);
	this->model->GetLinks()[6]->AddRelativeTorque(angular_torque);

	//Set velocity
	//this->model->GetJoints()[2]->SetVelocity(0, vel[0]);
	//this->model->GetJoints()[3]->SetVelocity(0, vel[1]);
	//this->model->GetJoints()[4]->SetVelocity(0, vel[2]);
	//this->model->GetJoints()[5]->SetVelocity(0, vel[3]);
	math::Vector3 angular_vel;
	/*angular_vel.Set(0,0,-25);
	this->model->GetLinks()[3]->SetAngularVel(angular_vel);
	angular_vel.Set(0,0,-25);
	this->model->GetLinks()[4]->SetAngularVel(angular_vel);
	angular_vel.Set(0,0,25);
	this->model->GetLinks()[5]->SetAngularVel(angular_vel);
	angular_vel.Set(0,0,25);
	this->model->GetLinks()[6]->SetAngularVel(angular_vel);*/
}


	// Handle an incoming message from ROS
	// param[in] _msg A float value that is used to set the velocity of the Iris Rotors
	public: void OnRosMsg(const geometry_msgs::PoseConstPtr& msg)
	{
		this->flag_inicio = 1;

		/*this->iris_state_ref[0] = msg->x;
		this->iris_state_ref[1] = msg->y;
		this->iris_state_ref[2] = msg->z;
		std::cout << this->iris_state_ref[0] << " " << this->iris_state_ref[1] << " " << this->iris_state_ref[2] << "\n";
*/
		this->iris_state_ref[0] = msg->position.x;
		this->iris_state_ref[1] = msg->position.y;
		this->iris_state_ref[2] = msg->position.z;
		this->iris_state_ref[3] = msg->orientation.x;
		this->iris_state_ref[4] = msg->orientation.y;
		this->iris_state_ref[5] = msg->orientation.z;

	}

	// ROS helper function that processes messages
	private: void QueueThread()
	{
	  static const double timeout = 0.01;
	  while (this->rosNode->ok())
	  {
	    this->rosQueue.callAvailable(ros::WallDuration(timeout));
	  }
	}


	public: void control_callback(const ros::TimerEvent& event)
	{
	//ros::Time time = ros::Time::now();
	double time_ini = this->get_wall_time();
	double time_cpu_ini = this->get_cpu_time();	


	//this->model->GetWorldPose(); ---- Return the linear and angular position of Iris 
	//this->model->GetWorldPose().pos
	//this->model->GetWorldPose().rot.GetPitch();
	//this->model->GetWorldPose().rot.GetRoll();
	//this->model->GetWorldPose().rot.GetYaw();
	//this->model->GetWorldLinearVel(); ---- Return the linear velocity (xp, yp,zp)
	//this->model->GetWorldAngularVel(); ---- Return the angular velocity 
	
	// Get the current position and velocity
	math::Pose iris_pose = this->model->GetWorldPose();
	math::Vector3 iris_linear_vel, iris_angular_vel;
	iris_linear_vel = this->model->GetWorldLinearVel();
	iris_angular_vel = this->model->GetWorldAngularVel();
	
	// Update the state variables
	this->iris_state[0] = iris_pose.pos[0];
	this->iris_state[1] = iris_pose.pos[1];
	this->iris_state[2] = iris_pose.pos[2];
	this->iris_state[3] = iris_pose.rot.GetRoll();
	this->iris_state[4] = iris_pose.rot.GetPitch();
	this->iris_state[5] = iris_pose.rot.GetYaw();
	
	this->iris_state_vel[0] = iris_linear_vel[0];
	this->iris_state_vel[1] = iris_linear_vel[1];
	this->iris_state_vel[2] = iris_linear_vel[2];
	this->iris_state_vel[3] = iris_angular_vel[0];
	this->iris_state_vel[4] = iris_angular_vel[1];
	this->iris_state_vel[5] = iris_angular_vel[2];


	// Some prints just for debug
    //std::cout << iris_pose.rot.GetRoll() <<" "<< iris_pose.rot.GetPitch() <<" "<< this->iris_state[5] << "\n";
   // std::cout << this->iris_state_vel[3] <<" "<< this->iris_state_vel[4] << "\n";
	//std::cout << iris_angular_vel[0] <<" "<< iris_angular_vel[1] << "\n";
//	std::cout << this->iris_rotor_vel[0] <<" "<< this->iris_rotor_vel[1] <<" "<< this->iris_rotor_vel[2] <<" "<< this->iris_rotor_vel[3] << "\n\n";

	// Check the flag related to the first reference command
	if (this->flag_inicio){
	//Logic to ensure reference changes at the same time 
		this->tic_simu++;
		/*if (this->tic_simu == 1000)this->iris_state_ref[3] = 0.2;
		else if (this->tic_simu == 2000)this->iris_state_ref[4] = 0.3;
		else if (this->tic_simu == 3000)this->iris_state_ref[5] = -0.2;
		else if (this->tic_simu == 4000){
			this->iris_state_ref[2] = 1.5;
			this->iris_state_ref[3] = 0.1;
			this->iris_state_ref[4] = 0.2;
			this->iris_state_ref[5] = -0.3;
		}*/

		/*
		if (this->tic_simu == 1000) this->iris_state_ref[3] = 0.2;
		else if (this->tic_simu == 1500) this->iris_state_ref[4] = 0.3;
		else if (this->tic_simu == 2000) this->iris_state_ref[5] = -0.2;
		else if (this->tic_simu == 2500){
			this->iris_state_ref[3] = 0.1;
			this->iris_state_ref[4] = 0.15;
		}
		else if (this->tic_simu == 3000){
			this->iris_state_ref[3] = -0.2;
			this->iris_state_ref[4] = 0;
			this->iris_state_ref[5] = 0;
		}
		else if (this->tic_simu == 3500){
			this->iris_state_ref[2] = 1.5;
			this->iris_state_ref[3] = -0.3;
		}
		else if (this->tic_simu == 4000){
			this->iris_state_ref[4] = 0.2;
			this->iris_state_ref[5] = 0;
		}
		/*else if (this->tic_simu == 4500){
			this->iris_state_ref[3] = -0.2;
			this->iris_state_ref[]
		}*//*
		else if (this->tic_simu == 4500){
			this->iris_state_ref[2] = 2; 
			this->iris_state_ref[3] = -0.1;
			this->iris_state_ref[4] = -0.1;
			this->iris_state_ref[5] = 0.2;
		}	*/
		
		// Runs the MPC
		this->explicit_mpc();

		// Apply the new velocities
		///this->UpdateVelocity();
		this->delta_cpu_time = this->get_cpu_time() - time_cpu_ini;	

		// Publish the state and velocities to external analysis
		this->pub_data();

	}

	// Store old state
	this->old_iris_state[0] = this->iris_state[0];
	this->old_iris_state[1] = this->iris_state[1];
	this->old_iris_state[2] = this->iris_state[2];
	this->old_iris_state[3] = this->iris_state[3];
	this->old_iris_state[4] = this->iris_state[4];
	this->old_iris_state[5] = this->iris_state[5];
	this->old_iris_state_vel[0] = this->old_iris_state_vel[0];
	this->old_iris_state_vel[1] = this->old_iris_state_vel[1];
	this->old_iris_state_vel[2] = this->old_iris_state_vel[2];
	this->old_iris_state_vel[3] = this->old_iris_state_vel[3];
	this->old_iris_state_vel[4] = this->old_iris_state_vel[4];
	this->old_iris_state_vel[5] = this->old_iris_state_vel[5];

	/*
	//ros::Time time2 = ros::Time::now();
	std::cout << (this->get_wall_time() - time_ini) << "\n";
	//this->delta_cpu_time = this->get_cpu_time() - time_cpu_ini;
	std::cout << (this->get_cpu_time() - time_cpu_ini) << "\n";
	std::cout << (this->delta_cpu_time) << "\n\n";
*/
	}

	public: void pub_data(){
		
		// Create a variable to publish the state
		sensor_msgs::Imu pub_iris_state;

		common::Time cur_time = this->model->GetWorld()->GetSimTime();
		pub_iris_state.header.frame_id = "base_link";
		pub_iris_state.header.stamp.sec = cur_time.sec;
   		pub_iris_state.header.stamp.nsec = cur_time.nsec;


		//Rotor speed
		pub_iris_state.orientation.x = this->iris_rotor_vel[0];
		pub_iris_state.orientation.y = this->iris_rotor_vel[1];
		pub_iris_state.orientation.z = this->iris_rotor_vel[2];
		pub_iris_state.orientation.w = this->iris_rotor_vel[3];

		// Position data
		pub_iris_state.orientation_covariance[0] = this->iris_state[0];
		pub_iris_state.orientation_covariance[1] = this->iris_state[1];
		pub_iris_state.orientation_covariance[2] = this->iris_state[2];
		pub_iris_state.orientation_covariance[3] = this->iris_state[3];
		pub_iris_state.orientation_covariance[4] = this->iris_state[4];
		pub_iris_state.orientation_covariance[5] = this->iris_state[5];

		// Index Region
		pub_iris_state.orientation_covariance[7] = this->index_region ;

		// Time data
		pub_iris_state.orientation_covariance[8] = this->delta_cpu_time;

		// Velocity data
		pub_iris_state.angular_velocity_covariance[0] = this->iris_state_vel[0];
		pub_iris_state.angular_velocity_covariance[1] = this->iris_state_vel[1];
		pub_iris_state.angular_velocity_covariance[2] = this->iris_state_vel[2];
		pub_iris_state.angular_velocity_covariance[3] = this->iris_state_vel[3];
		pub_iris_state.angular_velocity_covariance[4] = this->iris_state_vel[4];
		pub_iris_state.angular_velocity_covariance[5] = this->iris_state_vel[5];

		//Control Action
		pub_iris_state.linear_acceleration_covariance[0] = this->control_action[0];
		pub_iris_state.linear_acceleration_covariance[1] = this->control_action[1];
		pub_iris_state.linear_acceleration_covariance[2] = this->control_action[2];
		pub_iris_state.linear_acceleration_covariance[3] = this->control_action[3];

		//Reference Value
		pub_iris_state.linear_acceleration_covariance[4] = this->iris_state_ref[2];
		pub_iris_state.linear_acceleration_covariance[5] = this->iris_state_ref[3];
		pub_iris_state.linear_acceleration_covariance[6] = this->iris_state_ref[4];
		pub_iris_state.linear_acceleration_covariance[7] = this->iris_state_ref[5];	
	
		geometry_msgs::Quaternion pub_vel_rotor;

		// Rotors data
		pub_vel_rotor.x = this->iris_rotor_vel[0];
		pub_vel_rotor.y = this->iris_rotor_vel[1];
		pub_vel_rotor.z = this->iris_rotor_vel[2];
		pub_vel_rotor.w = this->iris_rotor_vel[3];

		// Publish data
		this->state_pub.publish(pub_iris_state);
		this->vel_pub.publish(pub_vel_rotor);

	}


	//Controller function
	public: void explicit_mpc() {
		//Update parameter array for Explicit MPC
		this->state_array_explicit_mpc();

		//Runs the MPC 
		//int temp_index_region = index_region_evaluate_bst(this->control_state_array, &(this->bst_nodes));
		int temp_index_region = index_region_evaluate_bst(this->control_state_array, this->control_param.number_states,&(this->bst_nodes), &(this->ineq_set));

		//std::cout << "index region: " << temp_index_region << "\n";

		//Update index region only when find a region
		if (temp_index_region != 0) this->index_region = temp_index_region;
		else std::cout << "ruimmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm\n"; 			

		//calculate_control_explicit_bst(this->delta_control_action, this->control_state_array, &(this->regions), this->index_region - 1);
		calculate_control_explicit_bst(this->delta_control_action, this->control_state_array, this->control_param.number_states, this->control_param.number_controls_actions, &(this->regions), this->index_region - 1);


		//std::cout << this->control_action[0] << " " << this->control_action[1] << " " << this->control_action[2] << " " << this->control_action[3] << "\n";
		//std::cout << this->iris_state[5] << "\n\n"; 
		
		this->control_action[0] = this->delta_control_action[0] + this->old_control_action[0];
		this->control_action[1] = this->delta_control_action[1] + this->old_control_action[1];
		this->control_action[2] = this->delta_control_action[2] + this->old_control_action[2];
		this->control_action[3] = this->delta_control_action[3] + this->old_control_action[3];

		this->old_control_action[0] = this->control_action[0];
		this->old_control_action[1] = this->control_action[1];
		this->old_control_action[2] = this->control_action[2];
		this->old_control_action[3] = this->control_action[3];

		if (this->delta_control_action[0] > this->max_delta_control_action[0]) this->max_delta_control_action[0] = this->delta_control_action[0];
		if (this->delta_control_action[1] > this->max_delta_control_action[1]) this->max_delta_control_action[1] = this->delta_control_action[1];
		if (this->delta_control_action[2] > this->max_delta_control_action[2]) this->max_delta_control_action[2] = this->delta_control_action[2];
		if (this->delta_control_action[3] > this->max_delta_control_action[3]) this->max_delta_control_action[3] = this->delta_control_action[3];


		if (this->control_action[0] > this->max_control_action[0]) this->max_control_action[0] = this->control_action[0];
		if (this->control_action[1] > this->max_control_action[1]) this->max_control_action[1] = this->control_action[1];
		if (this->control_action[2] > this->max_control_action[2]) this->max_control_action[2] = this->control_action[2];
		if (this->control_action[3] > this->max_control_action[3]) this->max_control_action[3] = this->control_action[3];

		//std::cout << "delta max " <<  this->max_delta_control_action[0] << " " <<  this->max_delta_control_action[1] << " " <<  this->max_delta_control_action[2] << " " <<  this->max_delta_control_action[3] << "\n";
		//std::cout << "max " <<  this->max_control_action[0] << " " <<  this->max_control_action[1] << " " <<  this->max_control_action[2] << " " <<  this->max_control_action[3] << "\n\n";

		//Convert control action to rotors velocities 
		this->control_action_to_rotor_velocity();
	}

	public: void state_array_explicit_mpc(){
		//z
		this->control_state_array[0] = this->iris_state[2];
		//zp
		this->control_state_array[1] = this->iris_state_vel[2];
		//Phi
		this->control_state_array[2] = this->iris_state[3];
		//Phi p
		this->control_state_array[3] = this->iris_state_vel[3];
		//Theta
		this->control_state_array[4] = -this->iris_state[4];
		//Theta p
		this->control_state_array[5] = -this->iris_state_vel[4];
		//Psi
		this->control_state_array[6] = this->iris_state[5];
		//Psi p
		this->control_state_array[7] = this->iris_state_vel[5];
		//Old control action
		this->control_state_array[8] = this->old_control_action[0];
		this->control_state_array[9] = this->old_control_action[1];
		this->control_state_array[10] = this->old_control_action[2];
		this->control_state_array[11] = this->old_control_action[3];
		//Reference values
		this->control_state_array[12] = this->iris_state_ref[2];
		this->control_state_array[13] = this->iris_state_ref[3];
		this->control_state_array[14] = -this->iris_state_ref[4];
		this->control_state_array[15] = this->iris_state_ref[5];
	}

	public: void control_action_to_rotor_velocity() {
		double trust_z = this->control_action[0] + this->iris_mass*this->gravity;
		//std::cout << "trust z: " << trust_z << "\n";
		
		double vel_0_temp = (trust_z/(4*this->iris_KT) - this->control_action[1] / (4 * this->iris_KT*this->iris_ly) + this->control_action[2] / (4 * this->iris_KT*this->iris_lx) - this->control_action[3] / (4 * this->iris_KD));
		if (vel_0_temp > 0) this->iris_rotor_vel[0] = sqrt(vel_0_temp);
		else this->iris_rotor_vel[0] = 0;

		double vel_1_temp =( trust_z/(4*this->iris_KT) + this->control_action[1]/(4*this->iris_KT*this->iris_ly) - this->control_action[2]/(4*this->iris_KT*this->iris_lx) - this->control_action[3] / (4 * this->iris_KD));
		if (vel_1_temp > 0) this->iris_rotor_vel[1] = sqrt(vel_1_temp);
		else this->iris_rotor_vel[1] = 0;

		double vel_2_temp = (trust_z/ (4 * this->iris_KT) + this->control_action[1] / (4 * this->iris_KT*this->iris_ly) + this->control_action[2] / (4 * this->iris_KT*this->iris_lx) + this->control_action[3] / (4 * this->iris_KD));
		if (vel_2_temp > 0) this->iris_rotor_vel[2] = sqrt(vel_2_temp);
		else this->iris_rotor_vel[2] = 0;

		double vel_3_temp = (trust_z/ (4 * this->iris_KT) - this->control_action[1] / (4 * this->iris_KT*this->iris_ly) - this->control_action[2] / (4 * this->iris_KT*this->iris_lx) + this->control_action[3] / (4 * this->iris_KD));
		if (vel_3_temp > 0) this->iris_rotor_vel[3] = sqrt(vel_3_temp);
		else this->iris_rotor_vel[3] = 0;

		//std::cout << vel_0_temp << " " << vel_1_temp << " " << vel_2_temp << " " << vel_3_temp << "\n\n";

	}


	// Return the Wall time expended
	public: double get_wall_time(){
   		 struct timeval time;
   		 if (gettimeofday(&time,NULL)){
        	//  Handle error
       		return 0;
    	}
    		return (double)time.tv_sec + (double)time.tv_usec * .000001;
	}

	// Return the time expended by the CPU
	public: double get_cpu_time(){
    		return (double)clock() / CLOCKS_PER_SEC;
	}

	// A node used for transport
	private: transport::NodePtr node;

	// A subscriber to a named topic.
	private: transport::SubscriberPtr sub;

	// Pointer to the model.
	private: physics::ModelPtr model;

	// Model Name
	private: std::string model_name;

	// Pointer to the joint.
	private: physics::JointPtr joint;

	// A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;
	
	// A ROS subscriber
	private: ros::Subscriber rosSub;

	// Reference Subscriber Name
	private: std::string ref_sub_name;

	// Publisher of Iris States
	private: ros::Publisher state_pub;

	// State Publisher Name
	private: std::string state_pub_name;

	// Publisher of Iris Volicities
	private: ros::Publisher vel_pub;

	// Vel Cmd Publisher Name
	private: std::string vel_pub_name;

	// A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;

	// A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;  

	// Publisher Timer
	private: ros::Timer pubTimer;

	//Aerodynamics parameters
	private: double iris_KT = 0.000015670;

	private: double iris_KD = 0.0000002551;

	private: double iris_lx = 0.13;

	private: double iris_ly = 0.21;

	private: double iris_mass = 1.37;
	
	private: double gravity = 9.81;

	// Variables related to the currnte and old Irist State
	private: double iris_rotor_vel[4] = {0,0,0,0};
    
	private: double iris_state[6] = {0,0,0,0,0,0};
	
	private: double old_iris_state[6] = {0,0,0,0,0,0};

	private: double iris_state_vel[6] = {0,0,0,0,0,0};

	private: double old_iris_state_vel[6] = {0,0,0,0,0,0};

	//Controller Variables
	private: double iris_state_ref[6] = {0,0,0,0,0,0};

	private: double iris_state_vel_ref[6] = {0,0,0,0,0,0};

	private: double delta_control_action[4] = {0,0,0,0};

	private: double max_delta_control_action[4] = {0,0,0,0};

	private: double control_action[4] = {0,0,0,0};

	private: double old_control_action[4] = {0,0,0,0};

	private: double max_control_action[4] = {0,0,0,0};

	private: double control_state_array[16];

	//Explicit MPC data
	private: std::vector<struct_bst> bst_nodes;

	private: const char *filename_bst;// = "/home/schulze-ubuntu/iris_plugin_explicit_mpc/control_files/vt1_20_2_u3_linux/output_bst_20_2_u3_vt1.txt";

	private: std::vector<struct_regions> regions;

	private: const char *filename_regions;// = "/home/schulze-ubuntu/iris_plugin_explicit_mpc/control_files/vt1_20_2_u3_linux/output_regions_20_2_u3_vt1.txt";

	private: std::vector<struct_ineq_set> ineq_set;

	private: const char *filename_ineq_set;// = "/home/schulze-ubuntu/iris_plugin_explicit_mpc/control_files/vt1_20_2_u3_linux/output_ineq_20_2_u3_vt1.txt";
		
	private: struct_control_param control_param;

	private: const char *filename_control_param;// = "/home/schulze-ubuntu/iris_plugin_explicit_mpc/control_files/vt1_20_2_u3_linux/output_control_param_20_2_u3_vt1.txt";
		
	private: int index_region = 0;

	private: const char *filename_control_test;

    // A flag to initialize the controller only after the first reference command
	private: int flag_inicio = 0;

	// Store the time expended by the CPU
	private: double delta_cpu_time = 0;

	// Variable to update reference
	private: int tic_simu = 0;

	// Pointer to the update event connection
 	private: event::ConnectionPtr update_connection_;

	private: std::string plugin_pkg_path;

	};

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(IrisPluginExplicitMPC)
}
#endif
