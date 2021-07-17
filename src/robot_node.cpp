#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <iostream>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>

#include <visp_bridge/image.h>
#include <visp/vpImageTools.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/io/vpImageIo.h>


// добавить urdf
// перейти в новую конфигурацию
// сохранить конфигурацию 
// сохранить изображение

class RobotNode {

		int N;
		int RATE;
        bool isImageReceived;
		std::vector<double> q;

		std::string camera_topic;
		std::string js_state_topic;
		std::string urdf;
		std::string base_link;
		std::string tool_link;

		ros::NodeHandle nh;
		ros::Subscriber imageSub;
		ros::Subscriber js_state_Sub;

		vpImage<unsigned char> I;
		vpDisplayX d;

		vpCameraParameters cam;

		std::ofstream ur_poses_file;

public:

		/* Empty constructor */
		RobotNode() : nh("~") {
			N = 6;

            isImageReceived = false;
			ros::param::get("urdf_model", urdf);
			nh.param<int>("/rate", RATE, 50);
			nh.param<std::string>("/camera_topic", camera_topic, "/camera/color/image_raw");
			nh.param<std::string>("/js_state_topic", js_state_topic, "/joint_states");
			// nh.param<std::string>("/base_link", base_link, "base");
			// nh.param<std::string>("/tool_link", tool_link, "tool0");
			base_link = "base"
			tool_link = "tool0"

			double px = 605.8006591796875;
			double py = 606.1141967773438;
			double u0 = 317.1659240722656;
			double v0 = 235.714599609375;
			cam.initPersProjWithoutDistortion(px, py, u0, v0);
			std::cout << cam << std::endl;

			// std::cout << camera_topic << std::endl;
            // imageSub = nh.subscribe(camera_topic, 1, &RobotNode::image_callback, this);
			// js_state_Sub = nh.subscribe(js_state_topic, 1, &RobotNode::js_state_callback, this);

            imageSub = nh.subscribe("/camera/color/image_raw", 1, &RobotNode::image_callback, this);
			js_state_Sub = nh.subscribe("/joint_states", 1, &RobotNode::js_state_callback, this);


			for (int i = 0; i < N; ++i) { // WARN dim
				q.push_back(0);
			}

			ur_poses_file.open("~/ur_calibration_poses.txt");

            ROS_INFO("robot_node started");
			ROS_WARN("!!! Joint states dim are hardcoded to 6 (six)");
		}

		/* Destructor */
		~RobotNode() {
			ur_poses_file.close();
		}

		void image_callback(const sensor_msgs::Image::ConstPtr &msg) {
			I = visp_bridge::toVispImage(*msg);
			isImageReceived = true; // start main loop
		}

		void js_state_callback(const sensor_msgs::JointState::ConstPtr &msg) {
			for (int i = 0; i < N; i++) {	// WARN dim
				q[i] = msg->position[i];
			}
		}

		/* Extracts translation and rotation (roll, pitch, yaw) vectors from homogeneous matrix */
		void extract(vpHomogeneousMatrix M, vpTranslationVector &tr, vpQuaternionVector &qt) {
			M.extract(qt);
			M.extract(tr);
		}

		/* Waits while all instances will be initialized and simulation started*/
		void wait_for_image() {
			while (ros::ok()) {
                ROS_WARN("no image");
				if (isImageReceived)
					return;
				ros::spinOnce();
			}
		}

		/* The main ROS loop with big deals */
		void spin() {

			wait_for_image();

            d.init(I);
            vpDisplay::setTitle(I, "I am a window");

			// Generate kinematic model for orocos_kdl
			KDL::Tree tree;
			if (!kdl_parser::treeFromString(urdf, tree)) {
				printf("Failed to construct kdl tree\n");
			}
			KDL::Chain chain;
			if(!tree.getChain(base_link, tool_link, chain)){
				ROS_ERROR_STREAM("Failed to get KDL chain from tree ");
			}
			ROS_INFO("tip_name:  %s",tool_link.c_str());
			ROS_INFO("root_name: %s",base_link.c_str());
			
			// KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
			KDL::ChainFkSolverAcc fksolver = KDL::ChainFkSolverAcc(chain);

            unsigned cpt = 0;
            bool end = false;
			ros::Rate R(RATE);
			while (nh.ok() && !end) {
                
				vpDisplay::display(I);
				double t = vpTime::measureTimeMs();
				t = vpTime::measureTimeMs() - t;

                vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
                vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);

                vpMouseButton::vpMouseButtonType button;
                if (vpDisplay::getClick(I, button, false)) {

                    if (button == vpMouseButton::button1) {
                        cpt ++;
						
						KDL::JntArray q_kdl(N);
						for (int i = 0; i < N; ++i) {
							q_kdl(i) = q[i];
							std::cout << q[i] << " ";
						}
						std::cout << std::endl;

						KDL::Frame eeFrame;
						fksolver.JntToCart(q_kdl, eeFrame);

						double r, p, y;
						eeFrame.M.GetRPY(r, p, y);
						// std::cout << eeFrame.p.data[0] << eeFrame.p.data[1] << eeFrame.p.data[2] << std::endl;

                        vpPoseVector fPe(eeFrame.p.data[0], eeFrame.p.data[1], eeFrame.p.data[2], r, p, y);

                        std::stringstream ss_img, ss_pos;

                        ss_img << "image-" << cpt << ".png";
                        ss_pos << "pose_fPe_" << cpt << ".yaml";
                        std::cout << "Save: " << ss_img.str() << " and " << ss_pos.str() << std::endl;
                        vpImageIo::write(I, ss_img.str());
                        fPe.saveYAML(ss_pos.str(), fPe);
                    } else if (button == vpMouseButton::button3) {
                         end = true;
                    }
                }

				vpDisplay::flush(I);
				ros::spinOnce();
				R.sleep();
			}
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_node");

    try {
        RobotNode robot_node;
        robot_node.spin();
    } catch (int e){
        std::cout << "Restarted the node with new ros-sim-time" << std::endl;
    }
	return 0;
}
