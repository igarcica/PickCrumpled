#include "pick_crumpled_alg_node.h"

PickCrumpledAlgNode::PickCrumpledAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<PickCrumpledAlgorithm>(),
  kinova_linear_move_client_(private_node_handle_,"kinova_linear_move", true),
  as_(private_node_handle_, "activatesm", false)
  //action_name_("activatesm")
{

  this->state=IDLE;
  this->start_demo=false;
  this->start_experiments=false;
  this->stop=false;
  double open_gripper = 0.35;
  double close_gripper = 0.97; //0.81;
  // this->piling=false;
  this->n_obj_pile = 0;
  // this->expected_pile_thickn.push_back(0.0);
  this->object_thickness_drag = 0.055; //default towel 8l
  this->object_thickness_rotate = 0.07; //default towel 8l
  this->expected_pile_thickn = 0.0;
  this->placing_strategy="placevert";

  // Garment pose subscriber
  this->process_grasp_pointcloud = false;
  this->get_garment_position=false;
  this->get_garment_angle=false;
  this->get_garment_edge=false;
  //this->garment_angle_subscriber = this->public_node_handle_.subscribe("/segment_table/grasp_angle",1,&PickCrumpledAlgNode::garment_angle_callback,this);
  this->corners_subscriber = this->public_node_handle_.subscribe("/segment_table/pick_corners",1,&PickCrumpledAlgNode::corners_callback,this);
  this->pile_height_marker_subscriber = this->public_node_handle_.subscribe("/segment_table/pile_height_marker",1,&PickCrumpledAlgNode::pile_height_marker_callback,this);

  // Publish grasp marker
  this->garment_marker_publisher = this->public_node_handle_.advertise<visualization_msgs::Marker>("garment_marker", 1);
  this->grasp_marker_publisher = this->public_node_handle_.advertise<visualization_msgs::Marker>("grasp_marker", 1);

  // Publish handeye transform between ext_camera_link to base_link
  this->handeye_frame_pub_timer = this->public_node_handle_.createTimer(ros::Duration(1.0),&PickCrumpledAlgNode::handeye_frame_pub,this);
  this->handeye_frame_pub_timer.setPeriod(ros::Duration(1.0/50.0));
  this->handeye_frame_pub_timer.start();
  //this->handeye_frame_pub_timer.stop();

  this->get_params();

  this->pile_height = 0.0;
  std::cout << "\033[1;36m Pile height: -> \033[1;36m  x: " << this->pile_height << std::endl;

  this->pre_grasp_center.x = this->pre_grasp_corner[0]-0.08;
  this->pre_grasp_center.y = this->pre_grasp_corner[1];// - this->garment_width/2;
  this->pre_grasp_center.z = this->pre_grasp_corner[2];
  this->pre_grasp_center.theta_x = this->pre_grasp_corner[3];
  this->pre_grasp_center.theta_y = this->pre_grasp_corner[4];
  this->pre_grasp_center.theta_z = this->pre_grasp_corner[5];

  this->pre_grasp_pile_height_point.x = 0.53;
  this->pre_grasp_pile_height_point.y = 0.0;
  this->pre_grasp_pile_height_point.z = 0.4;
  this->pre_grasp_pile_height_point.theta_x = 179;
  this->pre_grasp_pile_height_point.theta_y = 0;
  this->pre_grasp_pile_height_point.theta_z = 90;

  // [init publishers]
  this->cartesian_velocity_publisher_ = this->private_node_handle_.advertise<kortex_driver::TwistCommand>("/" + this->robot_name + "/in/cartesian_velocity", 1);

  // [init subscribers]
  this->base_feedback_subscriber_ = this->private_node_handle_.subscribe("/" + this->robot_name  + "/base_feedback", 1000, &PickCrumpledAlgNode::base_feedback_callback, this);
  pthread_mutex_init(&this->base_feedback_mutex_,NULL);

  this->action_topic_subscriber_ = this->private_node_handle_.subscribe("/" + this->robot_name  + "/action_topic", 1000, &PickCrumpledAlgNode::action_topic_callback, this);
  pthread_mutex_init(&this->action_topic_mutex_,NULL);


  // [init services]


  // [init clients]
  exec_wp_trajectory_client_ = this->private_node_handle_.serviceClient<kortex_driver::ExecuteWaypointTrajectory>("/" + this->robot_name + "/base/execute_waypoint_trajectory");

  validate_waypoint_list_client_ = this->private_node_handle_.serviceClient<kortex_driver::ValidateWaypointList>("/" + this->robot_name + "/base/validate_waypoint_list");
  
  base_execute_action_client_ = this->private_node_handle_.serviceClient<kortex_driver::ExecuteAction>("/" + this->robot_name + "/base/execute_action");

  base_read_action_client_ = this->private_node_handle_.serviceClient<kortex_driver::ReadAction>("/" + this->robot_name + "/base/read_action");

  send_gripper_cmd_client_ = this->private_node_handle_.serviceClient<kortex_driver::SendGripperCommand>("/" + this->robot_name + "/base/send_gripper_command");

  set_cartesian_rf_client_ = this->private_node_handle_.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + this->robot_name + "/control_config/set_cartesian_reference_frame");

  base_clear_faults_client_ = this->private_node_handle_.serviceClient<kortex_driver::Base_ClearFaults>("/" + this->robot_name + "/base/clear_faults");

  activate_publishing_client_ = this->private_node_handle_.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + this->robot_name + "/base/activate_publishing_of_action_topic");



  // [init action servers]

  // PDDL variables
  this->plan_pddl_demo=false; //Start SM calling ROSPlan to generate plan
  this->pddl_demo=false;  //Ends the SM when the planned action is completed
  this->pddl_action_done=false;
  this->init_plan=true; //Predict deformation classes of listed objects before initial plan
  this->drag=false;
  this->rotate=false;
  this->rotation=90;
  this->nearest_edge="long";
  this->second_nearest_edge="short";
  this->workspace="grws";
  this->stiffness=0.0;
  this->friction=0.0;


  // [init action clients]


  ROS_DEBUG("PickCrumpledAlgNode:: Calling service activate_publishing_client_!");
  if (activate_publishing_client_.call(activate_publishing_srv_))
  {
    ROS_INFO("PickCrumpledAlgNode: Action notification activated!");
  }
  else
  {
    ROS_INFO("PickCrumpledAlgNode:: Failed to call service on topic %s",this->activate_publishing_client_.getService().c_str());
    this->success = false;
  }

  //*******************************************************************************
  // Make sure to clear the robot's faults else it won't move if it's already in fault
  this->success &= clear_faults();
  if (!this->success) exit(1);
  //*******************************************************************************

  this->success &= set_cartesian_reference_frame(kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED);
  if (!this->success) exit(1);

  // this->logfile("/home/userlab/Desktop/log_picknplace.txt", std::ios::app); 
  // std::ofstream logfile("/home/userlab/Desktop/log_picknplace.txt", std::ios::app); 
  logfile.open("/home/userlab/iri-lab/iri_ws/src/PickCrumpled/log_picknplace.txt", std::ios::app);
  this->logfile << "---------------------------------------\n";
  this->logfile << "\n ======= Initialized pick_crumpled_alg_node \n"; 
  // logfile.close();
  this->csvfile.open("/home/userlab/iri-lab/iri_ws/src/PickCrumpled/summary_picknplace.csv", std::ios::app);
  this->csvfile << "Object in pile, Object name, Predicted Class SHORT, Predicted Class LONG, Grasped edge, Sensed deformation, Placing Quality" << std::endl; //Headers
  this->planningfile.open("/home/userlab/iri-lab/iri_ws/src/PickCrumpled/planning_summary.txt", std::ios::app);
}


PickCrumpledAlgNode::~PickCrumpledAlgNode(void)
{
  this->logfile << "Closing pick_crumpled_alg_node\n";
  this->logfile.close(); //Close log file 
  // [free dynamic memory]
  pthread_mutex_destroy(&this->base_feedback_mutex_);
  pthread_mutex_destroy(&this->action_topic_mutex_);
}

void PickCrumpledAlgNode::mainNodeThread(void)
{
  //lock access to algorithm if necessary
  this->alg_.lock();
  ROS_DEBUG("PickCrumpledAlgNode::mainNodeThread");

  if(this->state!=IDLE && this->stop)
  {
    ROS_INFO("Demo Pick n Place has stopped!");
    this->state=IDLE;
    this->start_demo=false;
    this->start_experiments=false;
    this->stop=false;
  }
  else
  {
    switch(this->state)
    {
      case IDLE: ROS_DEBUG("PickCrumpledAlgNode: state IDLE");
                 if(this->start_demo)
                 {
                   ROS_INFO("PickCrumpledAlgNode (IDLE state): Opening gripper");
		               this->get_pile_height = false;
                   this->success &= send_gripper_command(this->open_gripper);
                   if (this->success)
                   {
                     this->state=PRE_PRE_ROTATE;
                     ros::Duration(0.5).sleep();
                     this->start_demo=false;
                   }else{
                     ROS_INFO("PickCrumpledAlgNode (IDLE state): Failed to open gripper");
                     this->start_demo=false;
                     this->state=IDLE;
                   }
                 }
                 else if(this->start_experiments)
		             {
		              // this->state=CHOOSE_PLACING;
                  this->state=EXPERIMENTS2;
		              this->start_experiments=false;
		             }
		             else
                   this->state=IDLE;
      break;

      // HOME POSITION
      case HOME: ROS_DEBUG("PickCrumpledAlgNode: state HOME");
                {
                  ROS_INFO("PickCrumpledAlgNode (HOME state): Moving to home position.");
                  this->logfile << "State: HOME" << std::endl;
                  this->success &= home_the_robot(); // Move the robot to the Home position with an Action
                  if (this->success)
                  {
                    this->state=PRE_PRE_ROTATE;
                    ros::Duration(0.5).sleep();
                  }else{
                    ROS_WARN("PickCrumpledAlgNode: Could not execute HOME action");
                    this->state=IDLE;
                  }
                }
      break;

      case PRE_PRE_ROTATE: ROS_DEBUG("PickCrumpledAlgNode: state PRE_PRE_ROTATE");
                          {
                            ROS_INFO("PickCrumpledSM: Sending to PRE_PRE_ROTATE position.");
                            this->logfile << "State: PRE_PRE_ROTATE" << std::endl;
                            std::cout << "\033[1;36m PRE-GRASP: -> \033[1;36m  x: " << this->pre_grasp_pile_height_point.x << ", y: " << this->pre_grasp_pile_height_point.y << ", z: " << this->pre_grasp_pile_height_point.z << std::endl;
                            this->success &= send_cartesian_pose(this->pre_grasp_pile_height_point);
                            if (this->success)
                            {
                              ROS_INFO("Success PRE PRE ROTATE");
                              this->state=PRE_ROTATE;
                              ros::Duration(0.5).sleep();
                            }
                            else
                              this->state=IDLE;
                          }
      break;

      case PRE_ROTATE:  ROS_DEBUG("PickCrumpledAlgNode: state PRE_ROTATE");
                        {
                          ROS_INFO("PickCrumpledSM: Sending to PRE_ROTATE position.");
                          this->logfile << "State: PRE_ROTATE" << std::endl;
                          kortex_driver::Pose current_grasp_pose = this->grasp_pile_height_point;
                          current_grasp_pose.z += 0.1; //3 cm above point
                          std::cout << "\033[1;36m PRE-GRASP: -> \033[1;36m  x: " << current_grasp_pose.x << ", y: " << current_grasp_pose.y << ", z: " << current_grasp_pose.z << std::endl;
                          this->success &= send_cartesian_pose(current_grasp_pose);
                          if (this->success)
                          {
                            ROS_INFO("Success PRE ROTATE");
                            this->state=ROTATE_POS;
                            ros::Duration(0.5).sleep();
                          }
                          else
                            this->state=IDLE;
                        }
      break;

      case ROTATE_POS: ROS_DEBUG("PickCrumpledAlgNode: state DRAG_ROTATE_POS");
                      {
                        ROS_INFO("PickCrumpledSM: Sending to DRAG_ROTATE position.");
                        this->logfile << "State: ROTATE_POS" << std::endl;
                        this->grasp_pile_height_point.z = this->grasp_pile_height_point.z + 0.05;
                        std::cout << "\033[1;36m PRE-GRASP: -> \033[1;36m  x: " << this->grasp_pile_height_point.x << ", y: " << this->grasp_pile_height_point.y << ", z: " << this->grasp_pile_height_point.z << std::endl;
                        this->success &= send_cartesian_pose(this->grasp_pile_height_point);
                        if (this->success)
                        {
                          ROS_INFO("Success ROTATE POS");
                          this->state=CLOSE_GRIPPER;
                          ros::Duration(0.5).sleep();
                        }
                        else
                          this->state=IDLE;
                      }
      break;


      // CLOSE GRIPPER
      case CLOSE_GRIPPER: ROS_DEBUG("PickCrumpledAlgNode: state CLOSE GRIPPER");
                          this->success &= send_gripper_command(this->close_gripper);
                          if (this->success)
                          {
                            this->state=POST_GRASP;
                            ros::Duration(0.5).sleep();
                          }
      break;

      case POST_GRASP: ROS_DEBUG("PickCrumpledAlgNode: state POST GRASP");
                       {
                          this->success &= home_the_robot(); // Move the robot to the Home position with an Action
                          if (this->success)
                          {
                            this->state=IDLE;
                            ros::Duration(0.5).sleep();
                          }else{
                            ROS_WARN("PickCrumpledAlgNode: Could not execute HOME action");
                            this->state=POST_GRASP;
                          }
                       }
      break;
                  

      // // POST-GRASP POSITION
      // // Sets a post grasp position a bit (x1.2) more high than the width of the garment
      // case POST_GRASP: ROS_DEBUG("PickCrumpledAlgNode: state POST GRASP");
      //                  {
      //                    this->logfile << "State: POST_GRASP" << std::endl;
      //                    this->success &= send_gripper_command(0.98);
      //                    ROS_INFO("PickCrumpledSM: Sending to POST-grasp position.");
      //                    geometry_msgs::Pose desired_pose;
      //                    desired_pose.position.x = tool_pose.x;
      //                    desired_pose.position.y = tool_pose.y;
      //                    desired_pose.position.z = 0.3;
      //                   //  std::cout << "\033[1;36m POST-GRASP: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
      //                    kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
      //                    this->state=WAIT_POST_GRASP;
      //                  }
      // break;

      // Waits until it reaches the post grasp position (linear movement controller)
      case WAIT_POST_GRASP: ROS_DEBUG("PickCrumpledAlgNode: state WAIT POST GRASP");
                            {
                              actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                              // to get the state of the current goal
                              this->alg_.unlock();
                              kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                              this->alg_.lock();

                              ROS_DEBUG("PickCrumpledAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                              if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                              {
                                ROS_INFO("Action aborted!");
                                this->state=END; //
                              }
                              else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                              {
                                this->success = true;
                                this->state=END;
                                ros::Duration(0.5).sleep();
                              }
                            }
      break;


      // OPEN GRIPPER
      case OPEN_GRIPPER:  ROS_DEBUG("PickCrumpledAlgNode: state OPEN GRIPPER");
			                    // if(config_.ok)
                          if(true)
			                    {
                            this->logfile << "State: OPEN_GRIPPER" << std::endl;
                            this->success &= send_gripper_command(this->open_gripper);
                            if (this->success)
                            {
                              this->state=POST_PLACE;
                              ros::Duration(0.5).sleep();
                            }
			                      else
			                      {
                              ROS_INFO("PickCrumpledAlgNode: Failed to open gripper");
			                        this->state=END;
			                      }
                            config_.ok=false;
			                    }
			                    else
			                      this->state=OPEN_GRIPPER;
      break;

      // POST-PLACE POSITION
      case POST_PLACE: ROS_DEBUG("PickCrumpledAlgNode: state POST PLACE");
                       {
                          ROS_INFO("PickCrumpledSM: Sending to POST-place position.");
                          this->logfile << "State: POST_PLACE" << std::endl;
                          geometry_msgs::Pose desired_pose;
                          desired_pose.position.x = tool_pose.x-0.07;
                          desired_pose.position.y = tool_pose.y;
                          desired_pose.position.z = tool_pose.z;
                          // std::cout << "\033[1;36m POST-PLACE: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                          kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
                          this->state=WAIT_POST_PLACE;
                       }
      break;

      case WAIT_POST_PLACE: ROS_DEBUG("PickCrumpledAlgNode: state WAIT POST PLACE");
                            {
                              actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                              // to get the state of the current goal
                              this->alg_.unlock();
                              kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                              this->alg_.lock();

                              ROS_DEBUG("PickCrumpledAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                              if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                              {
                                ROS_INFO("Action aborted!");
                                this->state=END;
                              }
                              else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                              {
                                this->success = true;
                                this->state=HIGH_POSITION;
                                ros::Duration(0.5).sleep();
                              }
                            }
      break;

      // HIGH POSITION
      case HIGH_POSITION: ROS_DEBUG("PickCrumpledAlgNode: state HIGH POSITION");
                          {
                            ROS_INFO("PickCrumpledSM: Sending to HIGH position.");
                            this->logfile << "State: HIGH_POSITION" << std::endl;
                            geometry_msgs::Pose desired_pose;
                            desired_pose.position.x = tool_pose.x;
                            desired_pose.position.y = tool_pose.y;
                            desired_pose.position.z = 0.35;
                            // std::cout << "\033[1;36m HIGH: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                            kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
                            this->state=WAIT_HIGH_POSITION;
                          }
      break;

      case WAIT_HIGH_POSITION: ROS_DEBUG("PickCrumpledAlgNode: state WAIT HIGH POSITION");
                               {
                                 actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                                 // to get the state of the current goal
                                 this->alg_.unlock();
                                 kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                                 this->alg_.lock();
                                 ROS_DEBUG("PickCrumpledAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                                 if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                                 {
                                   ROS_INFO("Action aborted!");
                                   this->state=END;
                                 }
                                 else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                                 {
                                   this->success = true;
                                   this->state=END;
                                   ros::Duration(0.5).sleep();
                                 }
                               }
      break;

      case END_POSITION: ROS_DEBUG("PickCrumpledAlgNode: state END POSITION");
                          {
                            ROS_INFO("PickCrumpledSM: Sending to END position.");
                            this->logfile << "State: END_POSITION" << std::endl;
                            geometry_msgs::Pose desired_pose;
                            desired_pose.position.x = 0.3;//0.4;//0.2;
                            desired_pose.position.y = -0.4;//0.5;
                            desired_pose.position.z = 0.4;
                            // std::cout << "\033[1;36m END: -> \033[1;36m  x: " << desired_pose.position.x << ", y: " <<  desired_pose.position.y << ", z: " << desired_pose.position.z << std::endl;
                            kinova_linear_moveMakeActionRequest(desired_pose, kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED, 0.08);
                            this->state=WAIT_END_POSITION;
                          }
      break;

      case WAIT_END_POSITION: ROS_DEBUG("PickCrumpledAlgNode: state WAIT END POSITION");
                               {
                                 actionlib::SimpleClientGoalState kinova_linear_move_state(actionlib::SimpleClientGoalState::PENDING);
                                 // to get the state of the current goal
                                 this->alg_.unlock();
                                 kinova_linear_move_state=kinova_linear_move_client_.getState(); // Possible state values are: PENDING,ACTIVE,RECALLED,REJECTED,PREEMPTED,ABORTED,SUCCEEDED and LOST
                                 this->alg_.lock();
                                 ROS_DEBUG("PickCrumpledAlgNode::mainNodeThread: kinova_linear_move_client_ action state = %s", kinova_linear_move_state.toString().c_str());;
                                 if(kinova_linear_move_state==actionlib::SimpleClientGoalState::ABORTED or kinova_linear_move_state==actionlib::SimpleClientGoalState::LOST)
                                 {
                                   ROS_INFO("Action aborted!");
                                   this->state=END;
                                 }
                                 else if(kinova_linear_move_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                                 {
                                   this->success = true;
                                   this->state=END;
                                   ros::Duration(0.5).sleep();
                                 }
                               }
      break;


      case END: ROS_INFO("PickCrumpledAlgNode: state END");
                {
                  this->logfile << "State: END" << std::endl;
                  this->stop=true;
                }
      break;

    }
  }


  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).


  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->cartesian_velocity_publisher_.publish(this->cartesian_velocity_TwistCommand_msg_);

  // Uncomment the following line to publish the topic message
  //this->my_gen3_action_topic_publisher_.publish(this->my_gen3_action_topic_ActionNotification_msg_);
  this->alg_.unlock();
}

// Gets config params
void PickCrumpledAlgNode::get_params(void)
{
  //init class attributes if necessary
  if(!this->private_node_handle_.getParam("rate", this->config_.rate))
  {
    ROS_WARN("PickCrumpledAlgNode::PickCrumpledAlgNode: param 'rate' not found");
  }
  else
    this->setRate(this->config_.rate);

  if(!this->private_node_handle_.getParam("robot_name", this->config_.robot_name))
  {
    ROS_WARN("PickCrumpledAlgNode::PickCrumpledAlgNode: param 'robot_name' not found");
  }
  else
    this->robot_name = this->config_.robot_name;

  // Definition parameter pose 00
   if(!this->private_node_handle_.getParam("pre_grasp_corner", this->pre_grasp_corner)) {
       ROS_WARN("PickCrumpledAlgNode::PickCrumpledAlgNode: param 'pre_grasp_corner' not found");
   } else {
       ROS_INFO("pre_grasp_corner: [%f, %f, %f, %f, %f, %f]", this->pre_grasp_corner[0], this->pre_grasp_corner[1], this->pre_grasp_corner[2], this->pre_grasp_corner[3], this->pre_grasp_corner[4], this->pre_grasp_corner[5]);
   }
  if(!this->private_node_handle_.getParam("close_gripper", this->config_.close_gripper))
  {
    ROS_WARN("PickCrumpledAlgNode::PickCrumpledAlgNode: param 'close_gripper' not found");
  }
  else
    this->close_gripper = this->config_.close_gripper;

  if(!this->private_node_handle_.getParam("garment_width", this->config_.garment_width))
  {
    ROS_WARN("PickCrumpledAlgNode::PickCrumpledAlgNode: param 'garment_width' not found");
  }
  else
    this->garment_width = this->config_.garment_width;

  // if(!this->private_node_handle_.getParam("garment_edge_size", this->config_.garment_edge_size))
  // {
  //   ROS_WARN("PickCrumpledAlgNode::PickCrumpledAlgNode: param 'garment_edge_size' not found");
  // }
  // else
  //   this->garment_edge_size = this->config_.garment_edge_size;

  if(!this->private_node_handle_.getParam("diagonal_place", this->config_.diagonal_place))
  {
    ROS_WARN("PickCrumpledAlgNode::PickCrumpledAlgNode: param 'diagonal_place' not found");
  }
  else
    this->diagonal_place = this->config_.diagonal_place;
}

void PickCrumpledAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  if(config.rate!=this->getRate())
    this->setRate(config.rate);

  // ---NAIVE APPROACH PARAMS---
  //Start SM for demo (use 'towel' bool to change strategy for grasping and placing towel (less gripper closure + vertical place) or napkin (more gripper closure + place2)
  if(config.start_demo)// && !config.plan_pddl_demo)
  {
    this->get_garment_position2=true;
    this->pddl_demo=false;
    this->start_demo=true;
    ROS_INFO("PickCrumpledAlgNode: Starting demo with selected gripper apperture and placing strategy");
    this->close_gripper=config.close_gripper;
    this->n_obj_pile = 0; //Place just one object
    this->objs_names.push_back(config.objs_to_pile); //obtain form reconfigure
    this->objs_layers.push_back(config.layers);
    if(config.vertical_place)
      {
        this->placing_strategy="placevert";//2
        ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Vertical");
      }
    else if(config.diagonal_place)
      {
        this->placing_strategy="placediag"; //1
        ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Diagonal");
      }
    else if(config.rotating_place)
      {
        this->placing_strategy="placerot"; //place2 3
        ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Rotating");
      }
    else if(config.dynamic_place)
      {
        this->placing_strategy="placedyn"; //dynamic (executed outside SM) //4
        ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Dynamic");
      }
    else
      {
        this->placing_strategy="placevert"; //2
        ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Vertical");
      }
    config.start_demo=false;
  }
  
  // Select grasping point
  if(config.get_grasp_point)
  {
    this->short_edge_sizes.push_back(config.garment_edge_size); //for placing positions in diagonal and rotating
    this->long_edge_sizes.push_back(config.garment_edge_size);
    this->process_grasp_pointcloud=true;
    this->get_garment_position=true;
    //this->get_garment_angle=true;
    config.get_grasp_point=false;
    // this->garment_edge_size=config.garment_edge_size;
  }

  // Execute Drag or Rotate actions before demo
  if(config.drag)
  {
    //ROS_WARN("PickCrumpledAlgNode: Activated PDDL SM management");
    this->drag=true;
    this->rotate=false;
  }
  if(config.rotate)
  {
    //ROS_WARN("PickCrumpledAlgNode: Activated PDDL SM management");
    this->drag=false;
    this->rotate=true;
  }

  // ---START SM FROM CHECK DEFORMATION---
  if(config.start_experiments) 
  {
    this->pddl_demo=false;
    this->start_experiments=true;
    this->n_obj_pile = 0; //Place just one object
    this->objs_names.push_back(config.objs_to_pile); //obtain form reconfigure
    this->objs_layers.push_back(config.layers);
    if(config.vertical_place)
    {
      this->placing_strategy="placevert"; //2
      ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Vertical");
    }
    else if(config.diagonal_place)
    {
      this->placing_strategy="placediag"; //1
      ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Diagonal");
    }
    else if(config.rotating_place)
    {
      this->placing_strategy="placerot"; //place2
      ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Rotating");
    }
    else
    {
      this->placing_strategy="placevert"; //2
      ROS_INFO("PickCrumpledAlgNode: Placing startegy --> Vertical");
    }
    config.start_experiments=false;
  }

  if(config.stop)
    this->stop=true;

  if(config.test)
  {
    this->pre_grasp_center.x = config.grasp_x;
    this->pre_grasp_center.y = config.grasp_y;
    this->pre_grasp_center.z = config.grasp_z;
    this->pre_grasp_center.theta_x = config.grasp_thetax;
    this->pre_grasp_center.theta_y = config.grasp_thetay;
    this->pre_grasp_center.theta_z = config.grasp_thetaz;
    std::cout << "\033[1;36m PRE-GRASP position: -> \033[1;0m  x: " << this->pre_grasp_center.x << ", y: " << this-> pre_grasp_center.y << ", z: " << this->pre_grasp_center.z << std::endl;
    std::cout << "\033[1;36m PRE-GRASP orientation: -> \033[1;0m  x: " << this->pre_grasp_center.theta_x << ", y: " << this-> pre_grasp_center.theta_y << ", z: " << this->pre_grasp_center.theta_z << std::endl;
    this->get_test_grasp_point = true;
    config.test=false;
  }
  config.ok=false;

  // this->config_=config;
  this->alg_.unlock();
}

 
/* PERCEPTION FUNCTIONS */
// Set and publish handeye transform
void PickCrumpledAlgNode::handeye_frame_pub(const ros::TimerEvent& event)
{
  // Set handeye transform
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(config_.handeye_x, config_.handeye_y, config_.handeye_z));
  tf::Quaternion q;
  q.setRPY(config_.handeye_r,config_.handeye_p,config_.handeye_yw);
  transform.setRotation(q);
  this->broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","ext_camera_link"));
}

// Subscribes to the topic sending the corner's position
// Renames them according to its position wrt base_link
void PickCrumpledAlgNode::corners_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  ROS_DEBUG("PickCrumpledAlgNode: Pick corners callback");
  if(this->process_grasp_pointcloud)
  {
    //Get distances to base_link and set corresponding names (down_left, up_right, etc)
    geometry_msgs::PointStamped point_in;
    geometry_msgs::PointStamped point_out;
    std::vector<geometry_msgs::PointStamped> points;

    // Transform point to robot base_link reference frame
    for(int i=0; i<msg->markers.size(); i++)
    {
      point_in.header.frame_id = msg->markers[i].header.frame_id;
      point_in.header.stamp = msg->markers[i].header.stamp;
      point_in.point = msg->markers[i].pose.position;

      this->listener.transformPoint("base_link", point_in, point_out);
      points.push_back(point_out);
    }

    // ---GET OBJECT THICKNESS---
    double max_z = std::max({points[0].point.z, points[1].point.z, points[2].point.z, points[3].point.z}); //Get the highest z value (wrt base_link)
    // std::cout << "OBJECT THICKNESS: " << max_z << std::endl;

    // ---GET CORNERS NAMES---
    geometry_msgs::Point corner_ul2, corner_dl2, corner_ur2, corner_dr2, center; //Better format, as we dont have orientation
    // Identify bottom-right (smallest x, smallest y wrt base_link) and top-left (largest x, largest y wrt base_link)
    corner_dr2 = points[0].point;
    corner_ul2 = points[0].point;
    for (int i = 1; i < 4; i++) {
      // if (points[i].point.x < corner_dr.x || (points[i].point.x == corner_dr.x && points[i].point.y < corner_dr.y))
      if (points[i].point.x <= corner_dr2.x && points[i].point.y <= corner_dr2.y)
          corner_dr2 = points[i].point;
      // if (points[i].point.x > corner_ul.x || (points[i].point.x == corner_ul.x && points[i].point.y > corner_ul.y))
      if (points[i].point.x >= corner_ul2.x && points[i].point.y >= corner_ul2.y)
          corner_ul2 = points[i].point;
    }
    // Identify the remaining two points
    geometry_msgs::Point remaining2[2];
    int idx2 = 0;
    for (int i = 0; i < 4; i++) {
      if (!(points[i].point.x == corner_dr2.x && points[i].point.y == corner_dr2.y) &&
          !(points[i].point.x == corner_ul2.x && points[i].point.y == corner_ul2.y)) {
          remaining2[idx2++] = points[i].point;
      }
    }
    // Assign top-right and bottom-left based on y-values wrt base_link
    if (remaining2[0].y > remaining2[1].y) {
        corner_dl2 = remaining2[0];
        corner_ur2 = remaining2[1];
    } else {
        corner_dl2 = remaining2[1];
        corner_ur2 = remaining2[0];
    }
    ROS_INFO("PickCrumpled: Identified corners:");
    ROS_INFO("Bottom Left  (DL): (%f, %f)", corner_dl2.x, corner_dl2.y);
    ROS_INFO("Bottom Right (DR): (%f, %f)", corner_dr2.x, corner_dr2.y);
    ROS_INFO("Top Left     (UL): (%f, %f)", corner_ul2.x, corner_ul2.y);
    ROS_INFO("Top Right    (UR): (%f, %f)", corner_ur2.x, corner_ur2.y);


    // ---GET CORNERS NAMES---
    geometry_msgs::Point corner_d1, corner_d2, corner_ul, corner_dl, corner_ur, corner_dr; //Better format, as we dont have orientation
    // Identify bottom points (smallest x wrt base_link) 
    corner_d1 = points[0].point; 
    corner_d2 = points[1].point;
    for (int i = 1; i < 4; i++) {
      if (points[i].point.x < corner_d1.x) //find lowest bottom point
      {
        corner_d2 = corner_d1;
        corner_d1 = points[i].point;
      }
      else if (points[i].point.x < corner_d2.x) //find two bottom points
      {
        corner_d2 = points[i].point;
      }
      if (corner_d1.y >= corner_d2.y) //Bottom point with higher y = Down left
      {
        corner_dl = corner_d1; 
        corner_dr = corner_d2;
      }else{
        corner_dl = corner_d2;
        corner_dr = corner_d1;
      }
    }
    // Identify the remaining two points (top points)
    geometry_msgs::Point remaining[2];
    int idx = 0;
    for (int i = 0; i < 4; i++) {
      if (!(points[i].point.x == corner_dr.x && points[i].point.y == corner_dr.y) &&
          !(points[i].point.x == corner_dl.x && points[i].point.y == corner_dl.y)) {
          remaining[idx++] = points[i].point;
      }
    }
    // Assign top-right and bottom-left based on y-values wrt base_link
    if (remaining[0].y > remaining[1].y) {
        corner_ul = remaining[0];
        corner_ur = remaining[1];
    } else {
        corner_ul = remaining[1];
        corner_ur = remaining[0];
    }
    ROS_INFO("PickCrumpled: Identified corners:");
    ROS_INFO("Bottom Left  (DL): (%f, %f)", corner_dl.x, corner_dl.y);
    ROS_INFO("Bottom Right (DR): (%f, %f)", corner_dr.x, corner_dr.y);
    ROS_INFO("Top Left     (UL): (%f, %f)", corner_ul.x, corner_ul.y);
    ROS_INFO("Top Right    (UR): (%f, %f)", corner_ur.x, corner_ur.y);

    // ---COMPUTE EDGES LENGTH AND CENTERS---
    // Define the edges 
    std::pair<geometry_msgs::Point, geometry_msgs::Point> edges[4] = {
        {corner_dl, corner_dr}, // Bottom edge
        {corner_ul, corner_ur}, // Top edge
        {corner_ul, corner_dl}, // Left edge
        {corner_dr, corner_ur}  // Right edge
    };

    // Compute the edges lenth and their midpoints
    double lengths[4];
    geometry_msgs::Point edge_centers[4];
    geometry_msgs::Point a, b;

    for (int i = 0; i < 4; i++) {
      a = edges[i].first;
      b = edges[i].second;
      lengths[i] = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
      edge_centers[i].x = (a.x + b.x) / 2.0;
      edge_centers[i].y = (a.y + b.y) / 2.0;
      ROS_DEBUG("Length: %d", lengths[i]);
      ROS_DEBUG("Center: (%d, %d )", edge_centers[i].x, edge_centers[i].y);
    }

    // ---GET NEAREST EDGE---
    // Find the nearest and second nearest center to the origin
    int nearestIndex = -1, secondNearestIndex = -1;
    double minDistance = 1000;
    double secondMinDistance = 1000;
    for (int i = 0; i < 4; i++) {
        double d = sqrt(pow(edge_centers[i].x, 2) + pow(edge_centers[i].y, 2));
        if (d < minDistance) {
          secondMinDistance = minDistance; // Update second nearest before updating the nearest
          secondNearestIndex = nearestIndex;
          minDistance = d;
          nearestIndex = i;
        } else if (d < secondMinDistance) {
          secondMinDistance = d;
          secondNearestIndex = i;
        }
    }
    // std::cout << "Nearest center to origin: (" << edge_centers[nearestIndex].x << ", " << edge_centers[nearestIndex].y << ") with distance " << minDistance << std::endl;
    // std::cout << "Second nearest center to origin: (" << edge_centers[secondNearestIndex].x << ", " << edge_centers[secondNearestIndex].y << ") with distance " << secondMinDistance << std::endl;



    // CENTER POINT OF GARMENT - computed averaging the coordinates of the corners
    geometry_msgs::Point garment_center;
    for (const auto& corner : points) { 
        garment_center.x += corner.point.x;
        garment_center.y += corner.point.y;
        garment_center.z += corner.point.z; // Its not necessary
    }
    garment_center.x /= points.size();
    garment_center.y /= points.size();
    garment_center.z /= points.size();
    double disGarmenCenter = sqrt(pow(garment_center.x, 2) + pow(garment_center.y, 2));

    
    if(this->get_garment_position)
    // if(config_.get_grasp_point) //for planner
    {
      //-------GRASPING POSITION-------
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x=0.01;
      marker.scale.y=0.01;
      marker.scale.z=0.01;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration();

      //Invertir edges si el planner indica que hay que coger el segundo edge y es reachable (ToDO. por ahora todo esto se hace con reconfigure)
      if(config_.grasp_second_edge)
      {
        std::cout << "Grasp second nearest edge" << std::endl;
        std::string temp = this->nearest_edge;
        this->nearest_edge=this->second_nearest_edge;
        this->second_nearest_edge=temp;
        int temp2 = nearestIndex;
        nearestIndex=secondNearestIndex;
        secondNearestIndex=temp2;
      }

      std::cout << "------------------------------------------------" << std::endl;
      std::cout << "Nearest center to origin: (" << edge_centers[nearestIndex].x << ", " << edge_centers[nearestIndex].y << ") with distance " << minDistance << std::endl;
      std::cout << "Second nearest center to origin: (" << edge_centers[secondNearestIndex].x << ", " << edge_centers[secondNearestIndex].y << ") with distance " << secondMinDistance << std::endl;
      std::cout << "Garment center: (" << disGarmenCenter << std::endl;

      // ---COMPUTE IF THE NEAREST EDGE IS LONG OR SHORT---
      if(lengths[nearestIndex] < lengths[secondNearestIndex]) 
      {
        this->nearest_edge="short";
        this->second_nearest_edge="long";
        this->garment_edge_size = this->long_edge_sizes[this->n_obj_pile]; //If nearest edge is short, the not_grasped_edge_size will be the long_edge_size
        ROS_WARN("PickCrumpledAlgNode: Nearest edge is SHORT");
      }else{
        this->nearest_edge="long";
        this->second_nearest_edge="short";
        this->garment_edge_size = this->short_edge_sizes[this->n_obj_pile]; //If nearest edge is long, the not_grasped_edge_size will be the short_edge_size
        ROS_WARN("PickCrumpledAlgNode: Nearest edge is LONG");
      }

        // ---COMPUTE THE GRASP ANGLE---
    // Compute perpendicular angle for the nearest edge
      double dx = edges[nearestIndex].first.x - edges[nearestIndex].second.x;
      double dy = edges[nearestIndex].first.y - edges[nearestIndex].second.y;
      double perpendicularAngle = std::atan2(dx, -dy); //Used to compute pregrasp distances
      double grasp_angle = std::atan2(dx, dy) * 180 / M_PI; //Used to compute grasp orientation - Angle wrt x-axis of base_link. horizontal edge=0, edge vert=-90, clockwise>0, anticlockwise<0
      std::cout << "Grasp angle: " << grasp_angle << std::endl;
      std::cout << "DX: x1= " << edges[nearestIndex].first.x << " - x2= " << edges[nearestIndex].second.x << std::endl;
      std::cout << "DY: y1= " << edges[nearestIndex].first.y << " - y2= " << edges[nearestIndex].second.y << std::endl;
      // std::cout << "Grasping perpendicular angle (radians): " << perpendicularAngle << std::endl;
      // std::cout << "Grasping perpendicular angle (degrees): " << perpendicularAngle * 180.0 / M_PI << std::endl;
      // std::cout << "N: " << std::atan2(dx, dy) * 180.0 / M_PI << std::endl; //Angle wrt x-axis of base_link. horizontal edge=0, edge vert=-90, clockwise>0, anticlockwise<0
      // std::cout << "A: " << std::atan2(dx, -dy) * 180.0 / M_PI << std::endl;
      // std::cout << "B: " << std::atan2(-dx, dy) * 180.0 / M_PI << std::endl;
      // std::cout << "AA: " << std::atan2(-dy, dx) * 180.0 / M_PI << std::endl;
      // float cos_alpha = (abs(dy))/(sqrt(pow(dx,2)+pow(dy,2)));
      // float alpha = acos(cos_alpha);
      // std::cout << "alpha: " << alpha << std::endl;
      // double grasp_angle = -std::atan2(-dy, dx);
      // std::cout << "Grasping angle (radians): " << grasp_angle << std::endl;
      // std::cout << "Grasping angle (degrees): " << grasp_angle * 180.0 / M_PI << std::endl;
    
      // Get current grasp position
      this->compute_grasp_angle(grasp_angle);
      // std_msgs::Float64 hola;
      // hola.data = 0;
      // this->compute_grasp_angle(hola);

      //---COMPUTE PRE GRASP POINT bsaed on perpendicular angle---
      this->pre_grasp_center.x = edge_centers[nearestIndex].x + 0.05 * cos(perpendicularAngle);
      this->pre_grasp_center.y = edge_centers[nearestIndex].y - 0.05 * sin(-perpendicularAngle);
      this->pre_grasp_distance.x = abs(0.05 * cos(perpendicularAngle));
      this->pre_grasp_distance.y = 0.05 * sin(-perpendicularAngle);
      std::cout << "\033[1;36m SECOND GRASP POINT -->  x: " <<  edge_centers[nearestIndex].x << " y: " << edge_centers[nearestIndex].y << "\033[1;0m" <<std::endl;
      std::cout << "pre grasp dist: (" << this->pre_grasp_distance.x << ", " << this->pre_grasp_distance.y << ")" << std::endl;
      // std::cout << "pre grasp position: (" << this->pre_grasp_center.x << ", " << this->pre_grasp_center.y << ")" << std::endl;

      // this->pre_grasp_center.x = edge_centers[nearestIndex].x-this->pre_grasp_distance.x;
      // this->pre_grasp_center.y = edge_centers[nearestIndex].y-this->pre_grasp_distance.y;
      this->pre_grasp_center.z = this->config_.table_height+0.05;
      this->grasping_point_garment = this->pre_grasp_center;

      std::cout << "\033[1;36m GRASP POINT -->  x: " <<  pre_grasp_center.x << " y: " << pre_grasp_center.y << " z: " << pre_grasp_center.z << "\033[1;0m" <<std::endl;
      std::cout << "\033[1;36m SECOND GRASP POINT -->  x: " <<  edge_centers[secondNearestIndex].x << " y: " << edge_centers[secondNearestIndex].y << "\033[1;0m" <<std::endl;
      marker.pose.position.x=pre_grasp_center.x;
      marker.pose.position.y=pre_grasp_center.y;
      marker.pose.position.z=0.005;// pre_grasp_center.z;
      grasp_marker_publisher.publish(marker); //Publish grasping point marker - nearest edge center
      ROS_WARN("test2");

      //--- GET PILE HEIGHT AND OBJECT'S EDGE SIZES ---
      this->pile_height = 0.0;
      this->get_pile_height = true;
      //this->garment_edge_size = garment_edge.data;
      this->grasped_edge_size = lengths[nearestIndex];
      this->not_grasped_edge_size = lengths[secondNearestIndex];
      std::cout << "\033[1;36m Non grasped edge size --> \033[1;0m " <<  this->garment_edge_size << std::endl;
      std::cout << "\033[1;36m SENSED Non grasped edge size --> \033[1;0m " <<  lengths[secondNearestIndex] << std::endl;
      std::cout << "\033[1;36m SENSED Grasped edge size --> \033[1;0m " <<  lengths[nearestIndex] << std::endl;

      // --- GET DRAGGING AND ROTATING POSES ---
      // Dragging pose is inclined orientation over garment
      // this->dragging_pose_garment = this->pre_grasp_center;
      this->dragging_pose_garment.x = garment_center.x; // + this->pre_grasp_distance.x;
      this->dragging_pose_garment.y = garment_center.y; // - 0.05 * sin(-perpendicularAngle);
      this->dragging_pose_garment.z = 0.10; //0.031
      this->dragging_pose_garment.theta_x = 125; //0; //this->pre_grasp_center.theta_x;
      this->dragging_pose_garment.theta_y = -1.6; //0; //-125; //this->pre_grasp_center.theta_x;
      this->dragging_pose_garment.theta_z = 88.8; //90; //180; //this->pre_grasp_center.theta_x;

      if(edge_centers[nearestIndex].y < edge_centers[secondNearestIndex].y) //If the second nearest edge is further away than nearest corner:
      {
        ROS_INFO("nearest < second nearest");
        std::cout << "nearest " << edge_centers[nearestIndex].y << " second: " << edge_centers[secondNearestIndex].y << std::endl;
        this->end_dragging_pose = 0.0; //Drag to the right
      }
      else
      {
        ROS_INFO("nearest > second nearest");
        std::cout << "nearest " << edge_centers[nearestIndex].y << " second: " << edge_centers[secondNearestIndex].y << std::endl;
        this->end_dragging_pose = 0.3; // Drag to the left
      }

      // Rotating pose is vertical orientation over garment center
      this->rotating_pose_garment.x = garment_center.x;
      this->rotating_pose_garment.y = garment_center.y;
      this->rotating_pose_garment.z = 0.20;
      this->rotating_pose_garment.theta_x = 179; //-179.4
      this->rotating_pose_garment.theta_y = 0; //1
      this->rotating_pose_garment.theta_z = 90; //92.7
      std::cout << "\033[1;36m ROTATING pose --> x: " << this->rotating_pose_garment.x << " y: " << this->rotating_pose_garment.y << " z: " << this->rotating_pose_garment.z << "\033[1;0m" << std::endl;

      marker.pose.position.x=edge_centers[secondNearestIndex].x; //garment_center.x;
      marker.pose.position.y=edge_centers[secondNearestIndex].y; //garment_center.y;
      marker.pose.position.z=0.01; //garment_center.z;
      garment_marker_publisher.publish(marker);
      ROS_WARN("test3");

      // --- VALIDATE GOAL POSE ---
      ROS_INFO("Check grasp");
      kortex_driver::Waypoint waypoint;
      std::cout << "\033[1;36m GRASP pose --> x: " << this->pre_grasp_center.x << " y: " << this->pre_grasp_center.y << " z: " << this->pre_grasp_center.z << "\033[1;0m" << std::endl;
      std::cout << "\033[1;36m GRASP pose --> x: " << this->pre_grasp_center.theta_x << " y: " << this->pre_grasp_center.theta_y << " z: " << this->pre_grasp_center.theta_z << "\033[1;0m" << std::endl;
      waypoint = FillCartesianWaypoint(this->pre_grasp_center, 0);
      bool valid = validate_waypoint(waypoint);
      if(valid)
        ROS_WARN("good");
      ROS_INFO("Check rotate");
      std::cout << "\033[1;36m DRAGGING pose --> x: " << this->dragging_pose_garment.x << " y: " << this->dragging_pose_garment.y << " z: " << this->dragging_pose_garment.z << "\033[1;0m" << std::endl;
      std::cout << "\033[1;36m DRAGGING pose --> x: " << this->dragging_pose_garment.theta_x << " y: " << this->dragging_pose_garment.theta_y << " z: " << this->dragging_pose_garment.theta_z << "\033[1;0m" << std::endl;
      waypoint = FillCartesianWaypoint(this->dragging_pose_garment, 0);
      valid = validate_waypoint(waypoint);
      if(valid)
        ROS_WARN("good");
      
      config_.get_grasp_point = false;

      if(this->pddl_demo)
      {
        if(this->config_.ok)
        {
          // --- LOG OBJECT INFO ---
          this->logfile << "\n---OBJECT STATE INFO---" << std::endl;
          this->logfile << "Nearest edge size: " << lengths[nearestIndex] << " / Second nearest edge size: " << lengths[secondNearestIndex] << std::endl;
          this->logfile << "Nearest edge grasp point: (" << edge_centers[nearestIndex].x << ", " << edge_centers[nearestIndex].y << ") with distance " << minDistance << std::endl;
          this->logfile << "Second nearest edge point: (" << edge_centers[secondNearestIndex].x << ", " << edge_centers[secondNearestIndex].y << ") with distance " << secondMinDistance << std::endl;
          this->logfile << "Garment center point: (" << garment_center.x << ", " << garment_center.y << ") with distance " << disGarmenCenter << std::endl;
          this->logfile << "Grasp pose: (" <<  pre_grasp_center.x << ", " << pre_grasp_center.y << ", " << pre_grasp_center.z << ", " << pre_grasp_center.theta_x << ", " << pre_grasp_center.theta_y << ", " << pre_grasp_center.theta_z << ")" << std::endl;
          this->logfile << "Not grasped edge size: " << this->garment_edge_size << std::endl;
          this->logfile << "Pile height: " << this->pile_height << std::endl;
          // UPDATE workspace
          check_worspaces(disGarmenCenter); //Get workspace based on distance of garment center
          this->get_garment_position=false;
          this->state=UPDATE_INIT_ROSPLAN_KB; //Update predicates garment_at (workspace) and at_pose (nearest edge)
          this->config_.ok=false;
          this->process_grasp_pointcloud=false;
        }
      }
      else
      {
        this->process_grasp_pointcloud=false;
        this->get_garment_position=false;
      }
    }
  }
}

//void PickCrumpledAlgNode::check_worspaces(double garment_center, double grasp_point)
void PickCrumpledAlgNode::check_worspaces(double garment_center)
{
  //Grasp workspace (Can be dragged and grasped but not rotated)

  //Rotate workspace (can rotate but maybe not grasp)
  if(garment_center<0.7)
  {
    ROS_WARN("PickCrumpled: Rotating workspace!");
    this->workspace = "rotws";
  }
  else
  {
    ROS_WARN("PickCrumpled: Out of rotating workspace!");
    this->workspace = "grws";
  }
  //Grasp workspace (can grasp but maybe not rotate)
  // if(<grasp_point<)

  //Rotate workspace (Can be dragged, grasped and rotated)

}

//void PickCrumpledAlgNode::garment_angle_callback(const std_msgs::Float64::ConstPtr& msg)
// void PickCrumpledAlgNode::compute_grasp_angle(const std_msgs::Float64& garment_angle)
/*Based on the cloth nearest edge orientation, it sets the end-effector grasping orientation perpendicular to the edge
  *Edge perpendicular angle (grasping_angle) is wrt base_link x-axis. angles < 0 are anticlockwise grasps, angles > 0 are clockwise grasps
  *Roll and pitch are different due to euler angles singularities, woll angle (which is the one to modify) goes from 0 up for anticlockwise grasps and from 180 down for clockwise grasps
*/
void PickCrumpledAlgNode::compute_grasp_angle(double grasping_angle)
{
  ROS_DEBUG("PickCrumpledAlgNode: Computing grasping angle perpendicular to cloth edge");
  
  double new_grasping_angle;
  if(grasping_angle<0)
  {
    //Compute grasping orientation
    new_grasping_angle = round(abs(grasping_angle));
    // ROS_INFO("PickCrumpled: End-effector's grasping orientation (%d, %d, %f)", -178, -50, new_grasping_angle);
    std::cout << "\033[1;36m Grasping orientation --> (-178, -50, " << new_grasping_angle << ")\033[1;0m" << std::endl;
    this->pre_grasp_center.theta_x = -178;
    this->pre_grasp_center.theta_y = -50;
    this->pre_grasp_center.theta_z = new_grasping_angle;
    //Compute cloth rotation to bring second nearest edge closer
    this->rotation=90; //rotate anticlockwise
  }else if(grasping_angle>=0)
  {
    //Compute grasping orientation
    new_grasping_angle = 180.0-grasping_angle;
    new_grasping_angle = round(new_grasping_angle);
    // ROS_INFO("PickCrumpled: End-effector's grasping orientation (%d, %d, %f)", 0, -125, new_grasping_angle);
    std::cout << "\033[1;36m Grasping orientation --> (0, -125, " << new_grasping_angle << ")\033[1;0m" << std::endl;
    this->pre_grasp_center.theta_x = 0;
    this->pre_grasp_center.theta_y = -125;
    this->pre_grasp_center.theta_z = new_grasping_angle;
    //Compute cloth rotation to bring second nearest edge closer
    this->rotation=-90; //rotate clockwise
  }

  /*//if(this->get_garment_angle)
  //{
  if(garment_angle.data == 0)
  {
    std::cout << "Gripper orientation \033[1;36m HORIZONTAL \033[1;0m" <<  std::endl;
    // this->pre_grasp_distance.x = 0.06;
    // this->pre_grasp_distance.y = 0;
    this->pre_grasp_center.theta_x = 0;
    this->pre_grasp_center.theta_y = -125.5;
    this->pre_grasp_center.theta_z = 180;
  }
  else if(garment_angle.data == 1)
  {
    std::cout << "Gripper horientation \033[1;36m DIAG IZQUIERDA \033[1;0m" <<  std::endl;
    // this->pre_grasp_distance.x = 0.08;
    // this->pre_grasp_distance.y = -0.08;
    this->pre_grasp_center.theta_x = 0;
    this->pre_grasp_center.theta_y = -120;
    this->pre_grasp_center.theta_z = 135;
  }
  else if(garment_angle.data == 2)
  {
    std::cout << "Gripper horientation \033[1;36m DIAG DERECHA \033[1;0m" <<  std::endl;
    // this->pre_grasp_distance.x = 0.08;
    // this->pre_grasp_distance.y = 0.08;
    this->pre_grasp_center.theta_x = -176;
    this->pre_grasp_center.theta_y = -52;
    this->pre_grasp_center.theta_z = 37;
  }
  else if(garment_angle.data == 3)
  {
    std::cout << "Gripper horientation \033[1;36m VERTICAL \033[1;0m" << std::endl;
    // this->pre_grasp_distance.x = 0;
    // this->pre_grasp_distance.y = 0.04;
    this->pre_grasp_center.theta_x = -179; //-170;
    this->pre_grasp_center.theta_y = -50; //-52;
    this->pre_grasp_center.theta_z = 85; //55;
  }
  this->get_garment_angle=false;
  this->get_garment_position=true;*/

  // std::cout << "Defined orientation -->   x: " << this->pre_grasp_center.theta_x << ", y: " << this-> pre_grasp_center.theta_y << ", z: " << this->pre_grasp_center.theta_z << std::endl;
  // std::cout << "Pregrasp Distances -->   x: " << this->pre_grasp_distance.x << ", y: " << this-> pre_grasp_distance.y << ", z: " << std::endl;
  //}
}

// Subscribes to topic sending the pile height
void PickCrumpledAlgNode::pile_height_marker_callback(const visualization_msgs::Marker::ConstPtr& msg)
{
  if(this->get_garment_position2)
  {
    //Get distances to base_link and set corresponding names (down_left, up_right, etc)
    geometry_msgs::PointStamped point_in;
    geometry_msgs::PointStamped point_out;

    // Transform point to robot base_link reference frame
    point_in.header.frame_id = msg->header.frame_id;
    point_in.header.stamp = ros::Time(0);
    point_in.point = msg->pose.position;

    this->listener.transformPoint("base_link", point_in, point_out);

    this->grasp_pile_height_point.x = point_out.point.x;
    this->grasp_pile_height_point.y = point_out.point.y;
    this->grasp_pile_height_point.z = point_out.point.z;
    this->grasp_pile_height_point.theta_x = 179;
    this->grasp_pile_height_point.theta_y = 0;
    this->grasp_pile_height_point.theta_z = 90;

    std::cout << "LOWEST POINT: x= " << this->grasp_pile_height_point.x << " / y= " << this->grasp_pile_height_point.y << " / z= " << this->grasp_pile_height_point.z << std::endl;
    this->get_garment_position2=false;
  }
}

void PickCrumpledAlgNode::get_grasp_point(const geometry_msgs::PoseStamped grasp_pose)
{
  geometry_msgs::PoseStamped grasp_pose;
  kortex_driver::Pose grasp_pile_height_point;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x=0.01;
  marker.scale.y=0.01;
  marker.scale.z=0.01;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  if(this->get_test_grasp_point)
  {
    // this->pre_grasp_center
    marker.pose.position.x = this->pre_grasp_center.x;
    marker.pose.position.y = this->pre_grasp_center.y;
    marker.pose.position.z = this->pre_grasp_center.z;
    garment_marker_publisher.publish(marker);
  }
  

}
//------------------------------------------------------------------------------------
/*  [subscriber callbacks] */
void PickCrumpledAlgNode::base_feedback_callback(const kortex_driver::BaseCyclic_Feedback::ConstPtr& msg)
{
  //ROS_INFO("PickCrumpledAlgNode::base_feedback_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->base_feedback_mutex_enter();
  tool_pose.x = msg->base.tool_pose_x;
  tool_pose.y = msg->base.tool_pose_y;
  tool_pose.z = msg->base.tool_pose_z;
  tool_pose.theta_x = msg->base.tool_pose_theta_x;
  tool_pose.theta_y = msg->base.tool_pose_theta_y;
  tool_pose.theta_z = msg->base.tool_pose_theta_z;

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->base_feedback_mutex_exit();
}

void PickCrumpledAlgNode::base_feedback_mutex_enter(void)
{
  pthread_mutex_lock(&this->base_feedback_mutex_);
}

void PickCrumpledAlgNode::base_feedback_mutex_exit(void)
{
  pthread_mutex_unlock(&this->base_feedback_mutex_);
}

void PickCrumpledAlgNode::action_topic_callback(const kortex_driver::ActionNotification::ConstPtr& msg)
{
  //ROS_INFO("PickCrumpledAlgNode::action_topic_callback: New Message Received");
  this->last_action_notification_event = msg->action_event;
  //use appropiate mutex to shared variables if necessary
  //this->alg_.lock();
  //this->action_topic_mutex_enter();

  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->action_topic_mutex_exit();
}

void PickCrumpledAlgNode::action_topic_mutex_enter(void)
{
  pthread_mutex_lock(&this->action_topic_mutex_);
}

void PickCrumpledAlgNode::action_topic_mutex_exit(void)
{
  pthread_mutex_unlock(&this->action_topic_mutex_);
}


/*  [service callbacks] */
// void PickCrumpledAlgNode::testCallback(const boost::shared_ptr<const rosplan_dispatch_msgs::DispatchService::Response> &response)
// {
//   ROS_INFO("Hola");
// }

/*  [action callbacks] */
void PickCrumpledAlgNode::kinova_linear_moveDone(const actionlib::SimpleClientGoalState& state,  const iri_kinova_linear_movement::kinova_linear_movementResultConstPtr& result)
{
  this->alg_.lock();
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
    ROS_DEBUG("PickCrumpledAlgNode::kinova_linear_moveDone: Goal Achieved!");
  else
    ROS_INFO("PickCrumpledAlgNode::kinova_linear_moveDone: %s", state.toString().c_str());

  //copy & work with requested result
  this->alg_.unlock();
}

void PickCrumpledAlgNode::kinova_linear_moveActive()
{
  this->alg_.lock();
  //ROS_INFO("PickCrumpledAlgNode::kinova_linear_moveActive: Goal just went active!");
  this->alg_.unlock();
}

void PickCrumpledAlgNode::kinova_linear_moveFeedback(const iri_kinova_linear_movement::kinova_linear_movementFeedbackConstPtr& feedback)
{
  this->alg_.lock();
  //ROS_INFO("PickCrumpledAlgNode::kinova_linear_moveFeedback: Got Feedback!");

  bool feedback_is_ok = true;

  //analyze feedback
  //my_var = feedback->var;

  //if feedback is not what expected, cancel requested goal
  if( !feedback_is_ok )
  {
    kinova_linear_move_client_.cancelGoal();
    //ROS_INFO("PickCrumpledAlgNode::kinova_linear_moveFeedback: Cancelling Action!");
  }
  this->alg_.unlock();
}


/*  [action requests] */
bool PickCrumpledAlgNode::kinova_linear_moveMakeActionRequest(const geometry_msgs::Pose& desired_pose, const int& rf_frame, const float& max_vel)
{
  // std::cout << "Sending to: -> (" << desired_pose.position.x << ", " << desired_pose.position.y << ", " << desired_pose.position.z << ") " << std::endl;
  ROS_INFO("PickCrumpledAlgNode: Sending to -> (%f, %f, %f) ", desired_pose.position.x, desired_pose.position.y, desired_pose.position.z);
  this->logfile << "Sending to: -> (" << desired_pose.position.x << ", " << desired_pose.position.y << ", " << desired_pose.position.z << ") " << std::endl;

  // IMPORTANT: Please note that all mutex used in the client callback functions
  // must be unlocked before calling any of the client class functions from an
  // other thread (MainNodeThread).
  bool ok;
  // this->alg_.unlock();
  if(kinova_linear_move_client_.isServerConnected())
  {
    //ROS_INFO("PickCrumpledAlgNode::kinova_linear_moveMakeActionRequest: Server is Available!");
    //send a goal to the action server
    //kinova_linear_move_goal_.data = my_desired_goal;
    kinova_linear_move_goal_.goal_position = desired_pose;
    kinova_linear_move_goal_.reference_frame = rf_frame;
    kinova_linear_move_goal_.maximum_velocity = max_vel;
    kinova_linear_move_client_.sendGoal(kinova_linear_move_goal_,
                boost::bind(&PickCrumpledAlgNode::kinova_linear_moveDone,     this, _1, _2),
                boost::bind(&PickCrumpledAlgNode::kinova_linear_moveActive,   this),
                boost::bind(&PickCrumpledAlgNode::kinova_linear_moveFeedback, this, _1));
    ROS_DEBUG("PickCrumpledAlgNode::kinova_linear_moveMakeActionRequest: Goal Sent.");
    // ok=true;
    return true;
  }
  else
  {
    ROS_ERROR("PickCrumpledAlgNode::kinova_linear_moveMakeActionRequest: action server is not connected. Check remap or server presence.");
    // ok=false;
    return false;
  }
  // this->alg_.lock();
  return ok;
}

void PickCrumpledAlgNode::addNodeDiagnostics(void)
{
}

bool PickCrumpledAlgNode::clear_faults(void)
{
  // Clear the faults
  if (base_clear_faults_client_.call(base_clear_faults_srv_))
  {
    //ROS_INFO("PickCrumpledAlgNode:: base_clear_faults_client_ received a response from service server");
    ROS_DEBUG("PickCrumpledAlgNode:: Clear the faults");
  }
  else
  {
    ROS_INFO("PickCrumpledAlgNode:: Failed to call service on topic %s",this->base_clear_faults_client_.getService().c_str());
    return false;
  }
  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

// This function sets the reference frame to the robot's base
bool PickCrumpledAlgNode::set_cartesian_reference_frame(const int &cartesian_rf)
{
  set_cartesian_rf_srv_.request.input.reference_frame = cartesian_rf;
  this->cartesian_rf = cartesian_rf;
  ROS_DEBUG("PickCrumpledAlgNode:: Calling service set_cartesian_rf_client_!");
  if (set_cartesian_rf_client_.call(set_cartesian_rf_srv_))
  {
    ROS_DEBUG("Setting reference frame.");
  }
  else
  {
    ROS_INFO("PickCrumpledAlgNode:: Failed to call service on topic %s",this->set_cartesian_rf_client_.getService().c_str());
    return false;
  }
  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  return true;
}

bool PickCrumpledAlgNode::send_gripper_command(double value)
{
  // Initialize the request
  kortex_driver::Finger finger;
  finger.finger_identifier = 0;
  finger.value = value;
  send_gripper_cmd_srv_.request.input.gripper.finger.push_back(finger);
  send_gripper_cmd_srv_.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;
  ROS_DEBUG("PickCrumpledAlgNode:: Calling service send_gripper_cmd_client_!");
  if (send_gripper_cmd_client_.call(send_gripper_cmd_srv_))
  {
    ROS_DEBUG("The gripper command was sent to the robot.");
  }
  else
  {
    ROS_INFO("PickCrumpledAlgNode:: Failed to call service on topic % ",this->send_gripper_cmd_client_.getService().c_str());
    return false;
  }
  ROS_DEBUG("PickCrumpledAlgNode: Gripper command sended");
  send_gripper_cmd_srv_.request.input.gripper.finger.pop_back();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

bool PickCrumpledAlgNode::home_the_robot(void)
{
  this->last_action_notification_event = 0;
  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  base_read_action_srv_.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!base_read_action_client_.call(base_read_action_srv_))
  {
    ROS_INFO("PickCrumpledAlgNode:: Failed to call service on topic %s",this->base_read_action_client_.getService().c_str());
    return false;
  }

  // We can now execute the Action that we read
  base_execute_action_srv_.request.input = base_read_action_srv_.response.output;
  ROS_DEBUG("PickCrumpledAlgNode:: Calling service base_execute_action_client_!");
  if (base_execute_action_client_.call(base_execute_action_srv_))
  {
    ROS_DEBUG("The Home position action was sent to the robot.");
  }
  else
  {
    ROS_INFO("PickCrumpledAlgNode:: Failed to call service on topic %s",this->base_execute_action_client_.getService().c_str());
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  return wait_for_action_end_or_abort();
}

bool PickCrumpledAlgNode::validate_waypoint(kortex_driver::Waypoint waypoint)
{
  //compute_inverse_kinematics
  ROS_INFO("PickCrumpledAlgNode: Validating goal pose");

  kortex_driver::WaypointList trajectory;
  trajectory.duration = 0;
  trajectory.use_optimal_blending = false;
  trajectory.waypoints.push_back(waypoint);

  validate_waypoint_list_srv_.request.input = trajectory;
  if (!validate_waypoint_list_client_.call(validate_waypoint_list_srv_))
  {
    std::string error_string = "PickCrumpled: Failed to call ValidateWaypointList";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  else
    ROS_INFO("PickCrumpled: Point validated!");

  int error_number = validate_waypoint_list_srv_.response.output.trajectory_error_report.trajectory_error_elements.size();
  if(error_number!=0)
  {
    ROS_WARN("PickCrumpled: VALIDATE ERROR?");
    std::cout << error_number << std::endl;
    return false;
  }
  else
    ROS_WARN("VALID POINT");
  
  return true;
}

bool PickCrumpledAlgNode::send_cartesian_pose(const kortex_driver::Pose &goal_pose)
{
  ROS_DEBUG("PickCrumpledAlgNode: send_cartesian_pose function");
  ROS_INFO("PickCrumpledAlgNode: Sending to -> %f, %f, %f, %f, %f, %f) ", goal_pose.x, goal_pose.y, goal_pose.z, goal_pose.theta_x, goal_pose.theta_y, goal_pose.theta_z);
  // std::cout << "Sending to: -> (" << goal_pose.x << ", " << goal_pose.y << ", " << goal_pose.z << ", " << goal_pose.theta_x << ", " << goal_pose.theta_y << ", " << goal_pose.theta_z << ")" << std::endl;
  this->logfile << "Sending to: -> (" << goal_pose.x << ", " << goal_pose.y << ", " << goal_pose.z << ", " << goal_pose.theta_x << ", " << goal_pose.theta_y << ", " << goal_pose.theta_z << ")" << std::endl;

  this->last_action_notification_event = 0;

  // Validate goal pose
  kortex_driver::Waypoint waypoint;
  waypoint = FillCartesianWaypoint(goal_pose, 0);
  bool valid = validate_waypoint(waypoint);
  if(!valid)
  {
    std::string error_string = "PickCrumpledAlgNode: Failed to call validate goal pose";
    ROS_ERROR("%s", error_string.c_str());
    ROS_WARN("PickCrumpledAlgNode: Failed to call validate goal pose");
    return false;
  }

  exec_wp_trajectory_srv_.request.input.waypoints.clear();
  exec_wp_trajectory_srv_.request.input.waypoints.push_back(FillCartesianWaypoint(goal_pose, 0));
  // exec_wp_trajectory_srv_.request.input.waypoints.push_back(waypoint);
  exec_wp_trajectory_srv_.request.input.duration = 0;
  exec_wp_trajectory_srv_.request.input.use_optimal_blending = false;

  ROS_DEBUG("PickCrumpledAlgNode: Calling service exec_wp_trajectory_client_!");
  if (exec_wp_trajectory_client_.call(exec_wp_trajectory_srv_))
  {
    ROS_INFO("PickCrumpledAlgNode: The new cartesian pose was sent to the robot.");
  }
  else
  {
    ROS_INFO("PickCrumpledAlgNode:: Failed to call service on topic %s",this->exec_wp_trajectory_client_.getService().c_str());
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return wait_for_action_end_or_abort();
}

kortex_driver::Waypoint PickCrumpledAlgNode::FillCartesianWaypoint(const kortex_driver::Pose &goal_pose, float blending_radius)
{
  kortex_driver::Waypoint waypoint;
  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = goal_pose.x;
  cartesianWaypoint.pose.y = goal_pose.y;
  cartesianWaypoint.pose.z = goal_pose.z;
  cartesianWaypoint.pose.theta_x = goal_pose.theta_x;
  cartesianWaypoint.pose.theta_y = goal_pose.theta_y;
  cartesianWaypoint.pose.theta_z = goal_pose.theta_z;
  cartesianWaypoint.reference_frame =  kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE; //3
  cartesianWaypoint.blending_radius = blending_radius;

  waypoint.oneof_type_of_waypoint.cartesian_waypoint.push_back(cartesianWaypoint);

  return waypoint;
}

bool PickCrumpledAlgNode::rotate_end_effector(float rotation)
{
  ROS_INFO("PickCrumpledAlgNode: Sending joint positions");

  last_action_notification_event = 0;

  // Get the actual joints position, you could create a subscriber to listen to the base_feedback but here we only need the latest message in the topic though
  auto current_joint_pos = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");
  // std::vector<float> joints_home_values = {0, 15, 180, 230, 0, 55, 90}; //HOME joint values

  kortex_driver::WaypointList trajectory;
  kortex_driver::Waypoint waypoint;
  kortex_driver::AngularWaypoint angularWaypoint;

   // Angles to send the arm to current position, except end-effector
  for (unsigned int i = 0; i < 6; i++) //all joints except end-effector
  {
    angularWaypoint.angles.push_back(current_joint_pos->actuators[i].position);
  }

  //Position of end-effector (joint7)
  float endeffector_joint_pos = current_joint_pos->actuators[6].position + rotation;
  if(endeffector_joint_pos > 359)
  {
    endeffector_joint_pos = endeffector_joint_pos - 359;
  }
  angularWaypoint.angles.push_back(endeffector_joint_pos);

  // Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
  // If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
  int angular_duration = 0;
  angularWaypoint.duration = angular_duration;

  // Initialize Waypoint and WaypointList
  waypoint.oneof_type_of_waypoint.angular_waypoint.push_back(angularWaypoint);
  trajectory.duration = 0;
  trajectory.use_optimal_blending = false;
  trajectory.waypoints.push_back(waypoint);

  validate_waypoint_list_srv_.request.input = trajectory;
  if (!validate_waypoint_list_client_.call(validate_waypoint_list_srv_))
  {
    std::string error_string = "Failed to call ValidateWaypointList";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  int error_number = validate_waypoint_list_srv_.response.output.trajectory_error_report.trajectory_error_elements.size();
  static const int MAX_ANGULAR_DURATION = 30;
  std::cout << "error number: " << error_number << std::endl;

  while (error_number >= 1 && angular_duration < MAX_ANGULAR_DURATION)
  {
    ROS_INFO("PickCrumpledAlgNode: Retrying sending joint position");
    angular_duration++;
    std::cout << "angular duration: " << angular_duration << std::endl;
    trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration;

    validate_waypoint_list_srv_.request.input = trajectory;
    if (!validate_waypoint_list_client_.call(validate_waypoint_list_srv_))
    {
      std::string error_string = "Failed to call ValidateWaypointList";
      ROS_ERROR("%s", error_string.c_str());
      return false;
    }
    error_number = validate_waypoint_list_srv_.response.output.trajectory_error_report.trajectory_error_elements.size();
  }
  
  if (angular_duration >= MAX_ANGULAR_DURATION)
  {
    // It should be possible to reach position within 30s
    // WaypointList is invalid (other error than angularWaypoint duration)
    std::string error_string = "WaypointList is invalid";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Send the angles
  exec_wp_trajectory_srv_.request.input = trajectory;
  if (exec_wp_trajectory_client_.call(exec_wp_trajectory_srv_))
  {
    ROS_DEBUG("The joint angles were sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteWaypointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

bool PickCrumpledAlgNode::send_joint_angles(void)
{
  //72, 44.57, 155.64, 237.8, 55.95, 67.04, 356.1 
  last_action_notification_event = 0;

  kortex_driver::WaypointList trajectory;
  kortex_driver::Waypoint waypoint;
  kortex_driver::AngularWaypoint angularWaypoint;

  // // Angles to send the arm to vertical position (all zeros)
  // for (unsigned int i = 0; i < degrees_of_freedom; i++)
  // {
  //   angularWaypoint.angles.push_back(0.0);
  // }
  angularWaypoint.angles.push_back(72.0);
  angularWaypoint.angles.push_back(44.57);
  angularWaypoint.angles.push_back(155.64);
  angularWaypoint.angles.push_back(237.8);
  angularWaypoint.angles.push_back(55.95);
  angularWaypoint.angles.push_back(67.04);
  angularWaypoint.angles.push_back(356.1); //end-effector

  // Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
  // If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
  int angular_duration = 0;
  angularWaypoint.duration = angular_duration;

  // Initialize Waypoint and WaypointList
  waypoint.oneof_type_of_waypoint.angular_waypoint.push_back(angularWaypoint);
  trajectory.duration = 0;
  trajectory.use_optimal_blending = false;
  trajectory.waypoints.push_back(waypoint);

  validate_waypoint_list_srv_.request.input = trajectory;
  if (!validate_waypoint_list_client_.call(validate_waypoint_list_srv_))
  {
    std::string error_string = "Failed to call ValidateWaypointList";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  int error_number = validate_waypoint_list_srv_.response.output.trajectory_error_report.trajectory_error_elements.size();
  static const int MAX_ANGULAR_DURATION = 30;

  while (error_number >= 1 && angular_duration < MAX_ANGULAR_DURATION)
  {
    angular_duration++;
    trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration;

    validate_waypoint_list_srv_.request.input = trajectory;
    if (!validate_waypoint_list_client_.call(validate_waypoint_list_srv_))
    {
      std::string error_string = "Failed to call ValidateWaypointList";
      ROS_ERROR("%s", error_string.c_str());
      return false;
    }
    error_number = validate_waypoint_list_srv_.response.output.trajectory_error_report.trajectory_error_elements.size();
  }

  if (angular_duration >= MAX_ANGULAR_DURATION)
  {
    // It should be possible to reach position within 30s
    // WaypointList is invalid (other error than angularWaypoint duration)
    std::string error_string = "WaypointList is invalid";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  exec_wp_trajectory_srv_.request.input = trajectory;

  // Send the angles
  if (exec_wp_trajectory_client_.call(exec_wp_trajectory_srv_))
  {
    ROS_DEBUG("The joint angles were sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteWaypointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}


//REVISAR!! Quitar while, poner comprobacion dentro de sm, mirar gestion estado actions tiago modules
bool PickCrumpledAlgNode::wait_for_action_end_or_abort(void)
{
  ROS_DEBUG("PickCrumpledAlgNode: wait_for_action_end_or_abort");
  while (ros::ok())
  {
    if (this->last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_DEBUG("Received ACTION_END notification");
      return true;
    }
    else if (this->last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification");
      return false;
    }
    else
      ROS_DEBUG("Wait for action end or abort"); //print state

    ros::spinOnce();
  }
  return false;
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PickCrumpledAlgNode>(argc, argv, "pick_crumpled_alg_node");
}
