//ROS base:
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

//ROS interfaces
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "order_optimizer/msg/order_demand.hpp"


//C++ includes
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <limits.h>



struct OrderData {
    bool valid{false};
    int order_id;
    double cx;
    double cy;
    std::vector<int> products;
};


struct Part {
    std::string part_name;
    std::string product_parent;
    double cx;
    double cy;

};

struct Product {
    int id;
    std::string product_name;
    std::vector<Part> parts;
};

struct Station {

    std::vector<Part> pick_parts;
    geometry_msgs::msg::PoseStamped station_pose;

};


class NodeClass : public rclcpp::Node
{
    public:



    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_current_position_subscriber_;
    rclcpp::Subscription<order_optimizer::msg::OrderDemand>::SharedPtr next_order_subscriber_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr calculated_path_publisher_;


    std::shared_ptr<YAML::Node> products_parsed_yaml_ptr_;

    OnSetParametersCallbackHandle::SharedPtr parameter_handler_;
    std::map<std::string,rclcpp::Parameter> local_ros_parameters_ ;

    std::shared_ptr<std::vector<std::vector<double>>> path_planning_memory_ptr_;
    std::shared_ptr<std::vector<std::vector<int>>> path_planning_parent_ptr_;

    geometry_msgs::msg::PoseStamped last_recived_robot_pose_;

    NodeClass(std::string node_name) : Node(node_name,
                    rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)) {

        init_local_parameters();

        robot_current_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/currentPosition", 10, std::bind(&NodeClass::robot_current_position_subscriber_CB, this, std::placeholders::_1));
       
        next_order_subscriber_ =  this->create_subscription<order_optimizer::msg::OrderDemand>(
            "/nextOrder", 10, std::bind(&NodeClass::next_order_subscriber_CB, this, std::placeholders::_1));  


        calculated_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/order_path", 5);

        std::string product_list_path = local_ros_parameters_["local_system.database_path"].as_string() + "/configuration/products.yaml";
        
        parse_products_yaml(product_list_path);

        parameter_handler_ = this->add_on_set_parameters_callback(std::bind(&NodeClass::parametersCallback, this, std::placeholders::_1));
      
    }


    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";

      for (const auto &parameter : parameters)
      {

        auto whole_parameter_name = parameter.get_name();

        local_ros_parameters_[whole_parameter_name] = rclcpp::Parameter{parameter.get_name(),parameter.get_parameter_value()};


      }
  
      return result;
    }

    void init_local_parameters()
    {
        auto list_result = this->list_parameters(std::vector<std::string>{},0);

        auto parameters_list = this->get_parameters(list_result.names);

        for (const auto &parameter : parameters_list)
        {

            auto whole_parameter_name = parameter.get_name();

            local_ros_parameters_.insert(std::pair<std::string,rclcpp::Parameter>{whole_parameter_name,rclcpp::Parameter{parameter.get_name(),parameter.get_parameter_value()}});
            
        }
    }

    void robot_current_position_subscriber_CB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_recived_robot_pose_ = *msg;
    }



    void next_order_subscriber_CB(const order_optimizer::msg::OrderDemand::SharedPtr msg)
    {   
        RCLCPP_INFO_STREAM(this->get_logger(),"Searching data for order ID: " << msg->order_id << std::endl);
        OrderData order = search_order_data(msg->order_id);

        if(order.valid == false)
        {   RCLCPP_WARN_STREAM(this->get_logger(),"No data found for order ID: " << msg->order_id << std::endl);
            return;
        } 

        std::shared_ptr<std::vector<Product>> products_data =  std::make_shared<std::vector<Product>>();


        RCLCPP_INFO_STREAM(this->get_logger(),"Found data for order ID: " << order.order_id );
        RCLCPP_INFO_STREAM(this->get_logger(),"Products: ");
        for (const auto& product : order.products) {
            RCLCPP_INFO_STREAM(this->get_logger(),product );
        }

        search_products_data(order,products_data);
            
        std::shared_ptr<std::vector<Station>> order_path =  std::make_shared<std::vector<Station>>();
        search_for_optimal_path(order, last_recived_robot_pose_,  products_data, order_path );
        print_path_data(order, order_path ,msg->description);
        visualize_path( last_recived_robot_pose_, order_path, order);
    }

    OrderData search_order_data(int order_id)
    {   
        
        std::string database_path = local_ros_parameters_["local_system.database_path"].as_string() + "/orders";
 
     
        std::vector<std::string> file_paths_list{};
        for (const auto& entry : std::filesystem::directory_iterator(database_path)) {
              
            file_paths_list.push_back( entry.path());


        }

        std::vector<std::future<OrderData>> thread_vector;


        for(const auto & file_path : file_paths_list)
        {
            
            thread_vector.push_back(std::async(&NodeClass::parse_config_yaml, this, file_path, order_id, local_ros_parameters_["path_visualization.scale_factor"].as_double()));

            //parse_config_yaml(file_path, order_id);
        }

        bool wait_for_results{true};
        int finished_threads{0};
        int active_threads = int(thread_vector.size());
        std::chrono::milliseconds span(1);
        OrderData order;

        while(wait_for_results)
        { 
          int thread_i = 0;
          bool delete_thread{false};
          for(std::future<OrderData>& future : thread_vector)
          { 
            if(future.wait_for(span)==std::future_status::ready)
            {
              OrderData thread_order = future.get(); 
              finished_threads++;
              delete_thread = true;

              if(thread_order.valid == true)
              {
                order = thread_order;
              }
              break;
            }
            thread_i++;
    
          }

          if(delete_thread)
          {
            thread_vector.erase(thread_vector.begin()+thread_i);
          }
          

          if(active_threads == finished_threads)
          {
            wait_for_results = false;
          }


        }

        return order;

    }


    OrderData parse_config_yaml(const std::string file_path, int order_id, double dimension_scaling = 1)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Loading and parsing: " << file_path );
        OrderData order;
        YAML::Node orders = YAML::LoadFile(file_path);


        RCLCPP_INFO_STREAM(this->get_logger(),"Loading and parsing finished: " << file_path); 

        for (std::size_t i = 0; i < orders.size(); ++i) {

     
            if(orders[i]["order"].as<int>() == order_id)
            {
                
                
                order.valid = true;
                order.order_id = orders[i]["order"].as<int>();
                order.cx = orders[i]["cx"].as<double>()/dimension_scaling;
                order.cy = orders[i]["cy"].as<double>()/dimension_scaling;
                for (std::size_t j = 0; j < orders[i]["products"].size(); ++j) {
                    order.products.push_back(orders[i]["products"][j].as<int>());
                }

                return order;
            }
        

        }

        return order;


    }


    bool parse_products_yaml(const std::string file_path)
    {
        
        RCLCPP_INFO_STREAM(this->get_logger(),"Loading and parsing configuration file: " << file_path); 
        products_parsed_yaml_ptr_ = std::make_shared<YAML::Node>(YAML::LoadFile(file_path));

        return true;
 

    }

    bool search_products_data( const OrderData order_data, std::shared_ptr<std::vector<Product>> products_data)
    {


        for (const int desire_product : order_data.products) {

           

            for (YAML::iterator yaml_iterator = products_parsed_yaml_ptr_->begin();  yaml_iterator  != products_parsed_yaml_ptr_->end(); yaml_iterator++) {
                YAML::Node product_from_database = *yaml_iterator;
                if(product_from_database["id"].as<int>() == desire_product)
                {
                    Product product;
                    product.id = product_from_database["id"].as<int>();
                    product.product_name = product_from_database["product"].as<std::string>();

                    for (std::size_t j = 0; j < product_from_database["parts"].size(); ++j) {
                        Part part;
                        part.part_name = product_from_database["parts"][j]["part"].as<std::string>();
                        part.cx = product_from_database["parts"][j]["cx"].as<double>()/local_ros_parameters_["path_visualization.scale_factor"].as_double();
                        part.cy = product_from_database["parts"][j]["cy"].as<double>()/local_ros_parameters_["path_visualization.scale_factor"].as_double();
                        product.parts.push_back(part);
                    }
                    products_data->push_back(product);
                }

            }

            
        
        }
        return true;
    }

    bool search_for_optimal_path(const OrderData order_data, const geometry_msgs::msg::PoseStamped robot_position, std::shared_ptr<std::vector<Product>> products_data, std::shared_ptr<std::vector<Station>> order_path )
    {

        std::vector<Station> station_list = sort_parts_to_stations(*products_data);

        Station start_station;
        start_station.station_pose.pose = robot_position.pose;

        Station goal_station;
        goal_station.station_pose.pose.position.x = order_data.cx;
        goal_station.station_pose.pose.position.y = order_data.cy;


        station_list = optimize_station_order(start_station,goal_station,station_list);

        for(Station station : station_list)
        {
            order_path->push_back(station);
        }


        return true;
    }

    std::vector<Station> sort_parts_to_stations(const std::vector<Product> products_data)
    {
        std::vector<Station> stations;

        for(Product product_data : products_data)
        {
            for( Part part : product_data.parts)
            {
                bool add_new_station{true};
               
                for(Station& station : stations) // check if part is on already added station
                {   
                    if((part.cx == station.station_pose.pose.position.x)&&(part.cy == station.station_pose.pose.position.y))
                    {
                        add_new_station = false;

                        Part next_part = part;
                        next_part.product_parent = product_data.product_name;
                        station.pick_parts.push_back(next_part);

                        break;
                    }
                }

                if(add_new_station)
                {
                    Station picking_station;
                    Part next_part = part;
                    next_part.product_parent = product_data.product_name;
                    picking_station.station_pose.pose.position.x  = part.cx;
                    picking_station.station_pose.pose.position.y  = part.cy;
                    
                    picking_station.pick_parts.push_back(next_part);
  
                    stations.push_back(picking_station);
                }
                
      
            }
      
        }


        return stations;
    }

    std::vector<Station> optimize_station_order(Station start_station, Station end_station, std::vector<Station> unordered_stations )
    {

        std::vector<Station> ordered_stations;

        std::vector<Station> node_list;
        node_list.push_back(start_station);

        for( Station station : unordered_stations)
        {
            node_list.push_back(station);
        }

        
        node_list.push_back(end_station);


        int matrix_size = node_list.size();
        std::shared_ptr<std::vector<std::vector<double>>> distance_matrix_ptr =std::make_shared<std::vector<std::vector<double>>>(matrix_size, std::vector<double>(matrix_size));

        for(int i = 0; i < matrix_size; i++) {

            for (int j = 0; j < matrix_size; j++) {

                distance_matrix_ptr->at(i)[j] = calculate_station_distance(node_list[i],node_list[j]);

            }
        }
   

        path_planning_memory_ptr_ = std::make_shared<std::vector<std::vector<double>>>(matrix_size, std::vector<double>(1 << matrix_size, DBL_MAX));
        path_planning_parent_ptr_ = std::make_shared<std::vector<std::vector<int>>>(matrix_size, std::vector<int>(1 << matrix_size, -1));


        /*for (int i = 0; i < matrix_size; i++) {
            double solution =  search_recursive(i, (1 << (matrix_size )) - 1, distance_matrix_ptr);

            RCLCPP_INFO_STREAM(this->get_logger(),"solution " << solution);
        }*/
        search_recursive(matrix_size-1, (1 << (matrix_size )) - 1, distance_matrix_ptr);
       
        RCLCPP_INFO_STREAM(this->get_logger(), "Optimal path: ");
        std::shared_ptr<std::vector<int>> path = std::make_shared<std::vector<int>>();
        collect_path(matrix_size-1, (1 << (matrix_size)) - 1, path);
        

        for(size_t t = 1; t<path->size()-1; t++) // dont add start and goal
        {
            ordered_stations.push_back(node_list[path->at(t)]);
        }
 


        return ordered_stations;
    }

    double calculate_station_distance(const Station first_station, const Station second_station)
    {
   
        double x_side =  first_station.station_pose.pose.position.x -  second_station.station_pose.pose.position.x;
        double y_side =  first_station.station_pose.pose.position.y -  second_station.station_pose.pose.position.y;
                
        return std::sqrt(std::pow((x_side),2) + std::pow((y_side),2));
    }



    double search_recursive(int i, int mask, std::shared_ptr<std::vector<std::vector<double>>> distance_matrix_ptr)
    {
   
        if (mask == ((1 << i) | 1 ))
            return distance_matrix_ptr->at(0)[i];
  
        if (path_planning_memory_ptr_->at(i)[mask] != DBL_MAX)
            return path_planning_memory_ptr_->at(i)[mask];
    
        double res = DBL_MAX;
    
        int node_n = distance_matrix_ptr->size();
        for (int j = 0; j < node_n; j++)
        {
            if ((mask & (1 << j)) && j != i && j != 0)  //if j in mask is 1 not visited yet, not equal same joint , new proposed joint is not 0 (start joint), 
            {  
                double value = search_recursive(j, mask & (~(1 << i)), distance_matrix_ptr) + distance_matrix_ptr->at(j)[i];
                if(value < res)
                {   
                    res = value;
                    path_planning_parent_ptr_->at(i)[mask] = j;  
                   
                }                
            } 
        }                     
        path_planning_memory_ptr_->at(i)[mask] = res;                                
        return res;
    }


    void collect_path(int start, int mask, std::shared_ptr<std::vector<int>> path) {
        
        if (mask == 1) {
            RCLCPP_INFO_STREAM(this->get_logger(), "0");
            path->push_back(0);
            return;
        }
        int j = path_planning_parent_ptr_->at(start)[mask];
        
        collect_path(j, mask & (~(1 << start)), path);
        path->push_back(start);
        RCLCPP_INFO_STREAM(this->get_logger(), start );
      
    }

    bool print_path_data(const OrderData order_data, const std::shared_ptr<std::vector<Station>> order_path, const std::string description = "no description")
    {
        
        RCLCPP_INFO_STREAM(this->get_logger(),"Working on order: " << order_data.order_id << " (" << description << ")" );

        int counter{1};

        for( Station& station : *order_path)
        {   
            for( Part part : station.pick_parts)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), counter << ".\tFetching part \'" << part.part_name << 
                "\' for product \'" << part.product_parent << "\' at x: " << part.cx  << ", y: " << part.cy );
                counter++;
            }

            
        }
        RCLCPP_INFO_STREAM(this->get_logger(), counter << ".\tDelivering to destination x: " << order_data.cx  << ", y: " << order_data.cy );

        return true;
    }


    bool visualize_path(const geometry_msgs::msg::PoseStamped robot_position, const std::shared_ptr<std::vector<Station>> order_path, const OrderData order_data)
    {

        {
            // delete old visualization
            visualization_msgs::msg::MarkerArray marker_list;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = local_ros_parameters_["path_visualization.tf_frame_id"].as_string();
            marker.header.stamp = this->get_clock()->now();
            marker.action = marker.DELETEALL;
            marker_list.markers.push_back(marker);
            calculated_path_publisher_->publish(marker_list);
        }


        visualization_msgs::msg::MarkerArray marker_list;

        // add robot pose
        int marker_id{0};
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = local_ros_parameters_["path_visualization.tf_frame_id"].as_string();
        marker.header.stamp = this->get_clock()->now();
        marker.id = marker_id;
        marker_id++;
        marker.type = marker.CUBE;
        marker.action = marker.ADD;
        marker.pose = robot_position.pose;
        marker.ns = "parts";
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime.sec = 1000;
        marker_list.markers.push_back(marker);


        // add picking poses
        for (size_t part_i = 0; part_i < order_path->size(); part_i++ )
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = local_ros_parameters_["path_visualization.tf_frame_id"].as_string();
            marker.header.stamp = this->get_clock()->now();
            marker.id = marker_id;
            marker_id++;
            marker.type = marker.CYLINDER;
            marker.action = marker.ADD;
            marker.pose = order_path->at(part_i).station_pose.pose;
            marker.ns = "parts";
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.text = std::to_string(part_i);
            marker.lifetime.sec = 1000;
            marker_list.markers.push_back(marker);
        }

        // add destination pose

 
        //visualization_msgs::msg::Marker marker;
        marker.header.frame_id = local_ros_parameters_["path_visualization.tf_frame_id"].as_string();
        marker.header.stamp = this->get_clock()->now();
        marker.id = marker_id;
        marker_id++;
        marker.type = marker.CYLINDER;
        marker.action = marker.ADD;
        marker.pose.position.x = order_data.cx;
        marker.pose.position.y = order_data.cy;
        marker.ns = "parts";
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime.sec = 1000;
        marker_list.markers.push_back(marker);
    

        bool add_arows{true};
        if(add_arows)
        {   visualization_msgs::msg::MarkerArray arow_marker_list;

            for (size_t part_i = 0; part_i < marker_list.markers.size()-1; part_i++ )
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = local_ros_parameters_["path_visualization.tf_frame_id"].as_string();
                marker.header.stamp = this->get_clock()->now();
                marker.id = static_cast<int>(part_i);
                marker.type = marker.ARROW;
                marker.action = marker.ADD;
                marker.pose = marker_list.markers[part_i].pose;
                marker.ns = "path";
                double x_side = marker_list.markers[part_i+1].pose.position.x - marker.pose.position.x;
                double y_side = marker_list.markers[part_i+1].pose.position.y - marker.pose.position.y;
                
                tf2::Quaternion quaternion(tf2::Vector3(0,0,1), std::atan2(y_side,x_side));
                quaternion.normalize();
                marker.pose.orientation.w = quaternion.w();
                marker.pose.orientation.x = quaternion.x();
                marker.pose.orientation.y = quaternion.y();
                marker.pose.orientation.z = quaternion.z();
                marker.scale.x = std::sqrt(std::pow((x_side),2)
                                            + std::pow((y_side),2));
                marker.scale.y = 0.5;
                marker.scale.z = 0.5;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                marker.lifetime.sec = 1000;
                arow_marker_list.markers.push_back(marker);
                
            }
            calculated_path_publisher_->publish(arow_marker_list);
        }

        
        calculated_path_publisher_->publish(marker_list);

        return true;
    }




};





int main(int argc, char * argv[])
{
  // Define node name
  std::string node_name {"OrderOptimizer"};  

  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<NodeClass>(node_name);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  executor.spin();
 
  rclcpp::shutdown();
  return 0;
}