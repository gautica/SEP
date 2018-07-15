#include "../include/global_planner_qt/movebase.h"
#include "../include/global_planner_qt/grab.h"
#include "../include/global_planner_qt/param.h"
#include "../include/global_planner_qt/routing.h"

namespace global_planner {

    MoveBase::MoveBase() {
        sub_odom = nh.subscribe("/" + roboter_name + "/odom", 1, & MoveBase::odom_callback, this);
        sub_laser = nh.subscribe("/" + roboter_name + "/scan", 1, & MoveBase::scan_callback, this);
        pub_movement = nh.advertise < geometry_msgs::Twist > ("/" + roboter_name + "/cmd_vel", 1);

        moveDirection = FORWARD;
        rotateDirection = CLOCKWISE;
        speed = STOP;
        steps = LONG;
        isMoveable = true;
    }

    void MoveBase::init() {
        // Waiting bis roboter_pos position initialized
        std::cout << "Waiting for roboter position initialized ...\n";
        while (!is_roboter_pos_init) {
            ros::spinOnce();
        }
        std::cout << "Roboter position is initialized ...\n";

        // Init roboter local field
        int roboter_length = 4;
        roboter_local_field.resize(roboter_length);
        for (int i = 0; i < roboter_local_field.size(); i++) {
            roboter_local_field[i].resize(roboter_length);
        }
    }

    void MoveBase::start() {
        while (!is_map_init) {
            sleep(1);
        }
        init();

        excutePlan();
    }

    void MoveBase::excutePlan() {

        geometry_msgs::Twist twist;
        ros::Rate loop_rate(10);
        while (ros::ok()) {
          while (!stopSim && !is_job_finished && is_goal_ready) {
            while (request_new_plan) {
              sleep(1);
            }
            if (!reach_goal(twist) && isMoveable) {
                mutex.lock(); // lock, because path can be changed in MapThread
                find_nearst_pos();
                rotate(twist, 1.2);
                move(twist);
                ros::spinOnce();
                mutex.unlock();
                loop_rate.sleep();
            } else if (!isMoveable) {
              pubZeroVel(twist);
              mutex.lock(); // lock, because path can be changed in MapThread
              find_nearst_pos();
              rotate(twist, 0.02);
              mutex.unlock();
              //std::cout << "calc_movement(): " << calc_movement() << "\n";
              request_new_plan = true;
              sleep(1);
            }
          }
          pubZeroVel(twist);
          if (stopSim) {
            // Wait for starting ...
            mutex.lock();
            condition.wait( & mutex);
            mutex.unlock();
          }
        }
    }

    void MoveBase::odom_callback(const nav_msgs::OdometryConstPtr & msg) {
        // current linear position
        current_pose.x = msg -> pose.pose.position.x;
        current_pose.y = msg -> pose.pose.position.y;

        // current rotation
        tf::Quaternion quat(msg -> pose.pose.orientation.x,
            msg -> pose.pose.orientation.y,
            msg -> pose.pose.orientation.z,
            msg -> pose.pose.orientation.w);
        tf::Matrix3x3 matrix(quat);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        current_pose.theta = yaw;

        // initialize roboter_pos position as current position
        calc_coordinate_matrix(current_pose.x, current_pose.y, roboter_pos);
        is_roboter_pos_init = true;
    }

    void MoveBase::scan_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
        // Find the closest range between the defined minimum and maximum angles
        int minIndex = (int) ceil((MIN_SCAN_ANGLE_RAD - msg -> angle_min) / msg -> angle_increment);
        int maxIndex = (int) ceil(msg -> angle_max / msg -> angle_increment);

        float closest_range;
        if (moveDirection == FORWARD) {
            closest_range = msg -> ranges[0];
            for (int i = 0; i < minIndex; i++) {
                if (msg -> ranges[i] < closest_range) {
                    closest_range = msg -> ranges[i];
                }

                if (msg -> ranges[maxIndex - i - 2] < closest_range) {
                    closest_range = msg -> ranges[maxIndex - i - 2];
                }
            }
        } else {
            closest_range = msg -> ranges[maxIndex / 2 - minIndex];
            for (int i = maxIndex / 2 - minIndex; i <= maxIndex / 2 + minIndex; i++) {
                if (msg -> ranges[i] < closest_range) {
                    closest_range = msg -> ranges[i];
                }
            }
        }

        //ROS_INFO_STREAM("Closest Range: "  << closest_range);

        if (closest_range < MIN_PROXIMITY_RANGE_M)
        {
          ROS_INFO("seeing block ...");
          isMoveable = false;
          return;
        } else if (closest_range < 2 * MIN_PROXIMITY_RANGE_M) {
          ROS_ERROR("Warnning for block ...");
          speed = SLOW;
          steps = MEDIUM;
          isMoveable = true;
          return;
        }
        speed = FAST;
        steps = LONG;
        isMoveable = true;
    }

    void MoveBase::rotate(geometry_msgs::Twist & twist, double theta) {
        ros::Rate loop_rate(10);

        while (calc_movement() > theta) {

            twist.linear.x = 0;
            twist.angular.z = 2 * calc_movement() * rotateDirection;
            pub_movement.publish(twist);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void MoveBase::move(geometry_msgs::Twist & twist) {
        twist.angular.z = calc_movement() * rotateDirection;
        twist.linear.x = 0.01 * speed * moveDirection;
        pub_movement.publish(twist);
    }

    void MoveBase::rotate_to_goal(geometry_msgs::Twist & twist) {
        ros::Rate loop_rate(10);
        //double goal_yaw = curr_goal.yaw;
        int theta_x = curr_goal.goal_pos.second - roboter_pos.second;
        int theta_y = curr_goal.goal_pos.first - roboter_pos.first;
        double angle = atan2(theta_y, theta_x);
        double roboter_direction = current_pose.theta;
        if (angle < 0) {
            angle += 6.28;
        }
        if (roboter_direction < 0) {
            roboter_direction += 6.28;
        }



        ///rotate(twist, 0.05);
        while (abs(angle - roboter_direction) > 0.03) {
            twist.linear.x = 0;
            twist.angular.z = (angle - roboter_direction);
            pub_movement.publish(twist);
            ros::spinOnce();
            loop_rate.sleep();
            roboter_direction = current_pose.theta;
            if (roboter_direction < 0) {
                roboter_direction += 6.28;
            }
        }
        pubZeroVel(twist);
        loop_rate.sleep();
    }

    bool MoveBase::reach_goal(geometry_msgs::Twist & twist) {
        if (path.size() < curr_goal.distance_precision) {
            rotate_to_goal(twist);

          //sleep(1);
            if (goals.empty() && !request_new_plan) {
                Drop drop;
                drop.drop_resource();

                if (curr_product != NO_PRODUCT) {
                  curr_location = curr_product;
                  curr_product = machine_chain[machine_chain.size()-1];
                  machine_chain.pop_back();
                } else {
                  curr_location = WAREHOUSE;
                }

                Routing routing;
                routing.make_plan();
            } else if (!goals.empty()) {
                
                ROS_ERROR("%d", curr_goal.task);
                if (curr_goal.task == GRAB) {
                    Grab grab;
                    grab.grab_resource();
                }

                else if (curr_goal.task == DROP) {
                    Drop drop;
                    drop.drop_resource();
                }
                curr_goal = goals.front();//goals[goals.size() - 1];
                goals.pop();
                request_new_plan = true;   
            }
            while (request_new_plan) {
                sleep(1);
            }
            return true;
        }
        return false;
    }

    void MoveBase::pubZeroVel(geometry_msgs::Twist & twist) {
        twist.angular.z = 0;
        twist.linear.x = 0;
        pub_movement.publish(twist);
        ros::spinOnce();
    }

    void MoveBase::find_nearst_pos() {
        int count = path.size() - 1;
        while (calc_distance(path[count], roboter_pos) > calc_distance(path[count - steps], roboter_pos)) {
            path.resize(count - steps);
            count = path.size() - 1;
            if (count < steps) {
              break;
            }
        }

    }

    int MoveBase::calc_distance(std::pair < int, int > point1, std::pair < int, int > point2) {
        return signur(point1.first - point2.first) * (point1.first - point2.first) +
            signur(point1.second - point2.second) * (point1.second - point2.second);
    }

    void MoveBase::calc_coordinate_map(std::pair < int, int > & pixel, CoPair & coordinate) {
        int origin_x = COL / 2;
        int origin_y = ROW / 2;

        double x = ((double)(pixel.second - origin_x)) * mapResolution;
        double y = ((double)(pixel.first - origin_y)) * mapResolution;
        coordinate.first = x;
        coordinate.second = y;
    }

    void MoveBase::calc_coordinate_matrix(double x, double y, std::pair < int, int > & pixel) {
        int origin_x = COL / 2;
        int origin_y = ROW / 2;

        int i = origin_x + (int)(x / mapResolution);
        int j = origin_y + (int)(y / mapResolution);

        pixel.second = i;
        pixel.first = j;
    }

    float MoveBase::calc_movement() {
        int theta_x = path[path.size() - steps].second - roboter_pos.second;
        int theta_y = path[path.size() - steps].first - roboter_pos.first;
        float angle = atan2((double) theta_y, (double) theta_x);
        float theta = current_pose.theta;
        float PI = 3.1416;
        if (angle >= 0 && angle < PI / 2) {
            if (angle - PI / 2 < theta && theta <= angle) {
                rotateDirection = ANTICLOCKWISE;
                moveDirection = FORWARD;
                return this -> abs(theta - angle);
            } else if (angle < theta && angle + PI / 2 >= theta) {
                rotateDirection = CLOCKWISE;
                moveDirection = FORWARD;
                return this -> abs(theta - angle);
            } else if (angle - PI < theta && angle - PI / 2 >= theta) {
                rotateDirection = CLOCKWISE;
                moveDirection = BACKWARD;
                return this -> abs(theta - angle + PI); //--
            } else {
                rotateDirection = ANTICLOCKWISE;
                moveDirection = BACKWARD;
                if (theta > 0) {
                    return PI - theta + angle;
                } else {
                    return abs(theta - angle + PI);
                } //--
            }
        } else if ((angle >= PI / 2) && angle < PI) {
            if (angle - PI / 2 < theta && theta <= angle) {
                rotateDirection = ANTICLOCKWISE;
                moveDirection = FORWARD;
                return this -> abs(theta - angle);
            } else if (angle - PI < theta && angle - PI / 2 >= theta) {
                rotateDirection = CLOCKWISE;
                moveDirection = BACKWARD;
                return this -> abs(theta - angle + PI); //--
            } else if (angle - 3 * PI / 2 < theta && angle - PI > theta) {
                rotateDirection = ANTICLOCKWISE;
                moveDirection = BACKWARD;
                return this -> abs(theta - angle + PI); //--
            } else {
                rotateDirection = CLOCKWISE;
                moveDirection = FORWARD;
                if (theta < 0) {
                    return 2 * PI + theta - angle;
                } else {
                    return theta - angle;
                }
            }
        } else if (angle >= -(PI / 2) && angle < 0) {
            if (angle < theta && angle + PI / 2 >= theta) {
                rotateDirection = CLOCKWISE;
                moveDirection = FORWARD;
                return this -> abs(theta - angle);
            } else if (angle - PI / 2 < theta && theta < angle) {
                rotateDirection = ANTICLOCKWISE;
                moveDirection = FORWARD;
                return this -> abs(theta - angle);
            } else if (angle + PI / 2 < theta && angle + PI >= theta) {
                rotateDirection = ANTICLOCKWISE;
                moveDirection = BACKWARD;
                return this -> abs(theta - angle - PI); //--
            } else {
                rotateDirection = CLOCKWISE;
                moveDirection = BACKWARD;
                if (theta < 0) {
                    return PI + theta - angle;
                } else {
                    return theta - angle - PI;
                }
            }
        } else {
            if (angle < theta && angle + PI / 2 >= theta) {
                rotateDirection = CLOCKWISE;
                moveDirection = FORWARD;
                return this -> abs(theta - angle);
            } else if (angle + PI / 2 < theta && angle + PI >= theta) {
                rotateDirection = ANTICLOCKWISE;
                moveDirection = BACKWARD;
                return this -> abs(theta - angle - PI); //--
            } else if (angle + PI < theta && angle + 3 * PI / 2 >= theta) {
                rotateDirection = CLOCKWISE;
                moveDirection = BACKWARD;
                return this -> abs(theta - angle - PI);
            } else {
                rotateDirection = ANTICLOCKWISE;
                moveDirection = FORWARD;
                if (theta > 0) {
                    return 2 * PI - theta + angle;
                } else {
                    return abs(theta - angle);
                }
            }
        }
    }
} // namespace global_planner


