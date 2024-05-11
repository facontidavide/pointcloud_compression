#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>


#include "codec.hpp"
#include "zstd.h"

struct Stats
{
  int count = 0;

  long lossy_total_time_usec = 0;
  long lossless_total_time_usec = 0;

  double lossy_total_ratio = 0;
  double lossless_total_ratio = 0;
};



int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  rosbag2_cpp::Reader reader;
  reader.open(argv[1]);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

  std::map<std::string, Stats> stats;

  for (const auto& it: reader.get_all_topics_and_types())
  {
    if(it.type == "sensor_msgs/msg/PointCloud2")
    {
      stats[it.name] = Stats();
      std::cout << "Found PointCloud2 topic: " << it.name << std::endl;
    }
  }
 
  std::vector<char> lossy_buffer;
  std::vector<char> compressed_buffer;

  while (rclcpp::ok() && reader.has_next()) {

    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

    if (stats.count(msg->topic_name) == 0) 
    {
      continue;
    }

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    serialization.deserialize_message(&serialized_msg, ros_msg.get());

    const bool isXYZ = ros_msg->fields.size() == 3;
    const bool isXYZI = ros_msg->fields.size() == 4 && ros_msg->fields[3].name == "intensity";


    if(!isXYZ && !isXYZI)
    {
      std::cout << "Excluding " << msg->topic_name << " because we only support PointXYZ and PointXYZI currently" << std::endl;
      stats.erase(msg->topic_name);
      continue;
    }
   
    auto& stat = stats[msg->topic_name];
    stat.count++;
    // straightforward ZSTD compression of ros_msg->data
    {
      auto t_start = std::chrono::high_resolution_clock::now();

      int max_dst_size = ZSTD_compressBound(ros_msg->data.size());
      compressed_buffer.resize(max_dst_size);
      auto new_size = ZSTD_compress(compressed_buffer.data(), max_dst_size, ros_msg->data.data(), ros_msg->data.size(), 1);

      auto t_end = std::chrono::high_resolution_clock::now();
      auto usec = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

      stat.lossless_total_ratio += double(new_size) / double(ros_msg->data.size());
      stat.lossless_total_time_usec += usec;
    }
    // first step: lossy compression. 
    // second step: ZSTD compression
    {
      auto t_start = std::chrono::high_resolution_clock::now();
      const float resolution = 0.001;

      lossy_buffer.resize(ros_msg->data.size());
      auto* data_float = reinterpret_cast<float*>(ros_msg->data.data());
      size_t size_float = (ros_msg->data.size()) / sizeof(float);
      int lossy_size = isXYZ ?
          CompressPointXYZ(resolution, data_float, size_float, lossy_buffer.data()) : 
          CompressPointXYZI(resolution, data_float, size_float, lossy_buffer.data());
      lossy_buffer.resize(lossy_size);

      int max_dst_size = ZSTD_compressBound(lossy_buffer.size());
      compressed_buffer.resize(max_dst_size);
      auto new_size = ZSTD_compress(compressed_buffer.data(), max_dst_size, lossy_buffer.data(), lossy_buffer.size(), 1);

      auto t_end = std::chrono::high_resolution_clock::now();
      auto usec = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

      stat.lossy_total_ratio += double(new_size) / double(ros_msg->data.size());
      stat.lossy_total_time_usec += usec;
    }
  }

  for (const auto& [topic, stat]: stats)    
  {
    std::cout << "\nTopic: " << topic << std::endl;
    std::cout << "  Count: " << stat.count << std::endl;
    std::cout << "  Lossless Average time: " << stat.lossless_total_time_usec / stat.count << " usec" << std::endl;
    std::cout << "  Lossless Average ratio: " << stat.lossless_total_ratio / double(stat.count) << std::endl;
    std::cout << "  Lossy Average time: " << stat.lossy_total_time_usec / stat.count << " usec" << std::endl;
    std::cout << "  Lossy Average ratio: " << stat.lossy_total_ratio / double(stat.count) << std::endl;
  }

  return 0;
}
