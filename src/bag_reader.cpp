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
#include "lz4.h"

struct StatsData
{
  long total_time = 0;
  double total_ratio = 0;
};

struct Stats
{
  int count = 0;

  StatsData zstd;
  StatsData lz4;
  StatsData lossy;
  StatsData lossy_lz4;
  StatsData lossy_zstd;
};

int GetSizeZSTD(const std::vector<uint8_t>& input)
{
  static std::vector<char> compressed_buffer;
  int max_size = ZSTD_compressBound(input.size());
  compressed_buffer.resize(max_size);
  return ZSTD_compress(compressed_buffer.data(), max_size, input.data(), input.size(), 1);
}

int GetSizeLZ4(const std::vector<uint8_t>& input)
{
  static std::vector<char> compressed_buffer;
  int max_size = LZ4_compressBound(input.size());
  compressed_buffer.resize(max_size);
  return LZ4_compress_default((const char*)input.data(), compressed_buffer.data(), input.size(), max_size);
}


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
 
  std::vector<uint8_t> lossy_buffer;

  while (rclcpp::ok() && reader.has_next()) {

    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

    if (stats.count(msg->topic_name) == 0) 
    {
      continue;
    }

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    serialization.deserialize_message(&serialized_msg, ros_msg.get());

    PointType type = Undefined;
    if( ros_msg->fields.size() == 3 )
    {
      type = POINT_XYZ;
    }
    else if( ros_msg->fields.size() == 4 && ros_msg->fields[3].name == "intensity" )
    {
      type = POINT_XYZI;
    }
    else
    {
      std::cout << "Excluding " << msg->topic_name << " because the point type is not supported" << std::endl;
      stats.erase(msg->topic_name);
      continue;
    }

   
    auto& stat = stats[msg->topic_name];
    stat.count++;

    auto DurationUsec = [](const auto& diff) -> long
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
    };

    // straightforward ZSTD compression of ros_msg->data
    {
      auto t1 = std::chrono::high_resolution_clock::now(); 
      int new_size = GetSizeZSTD(ros_msg->data);
      auto t2 = std::chrono::high_resolution_clock::now();

      stat.zstd.total_ratio += double(new_size) / double(ros_msg->data.size());
      stat.zstd.total_time += DurationUsec(t2 - t1);
    }
    // straightforward LZ4 compression of ros_msg->data
    {
      auto t1 = std::chrono::high_resolution_clock::now(); 
      int new_size = GetSizeLZ4(ros_msg->data);
      auto t2 = std::chrono::high_resolution_clock::now();

      stat.lz4.total_ratio += double(new_size) / double(ros_msg->data.size());
      stat.lz4.total_time += DurationUsec(t2 - t1);
    }
    // Lossy + compressions
    const float resolution = 0.001;
    {
      auto t1 = std::chrono::high_resolution_clock::now();
      
      lossy_buffer.resize(ros_msg->data.size());
      int lossy_size = LossyPointcloudCompression(type, resolution, ros_msg->data.data(), ros_msg->data.size(), lossy_buffer.data());
      lossy_buffer.resize(lossy_size);

      auto t2 = std::chrono::high_resolution_clock::now();
      auto new_size_zstd = GetSizeZSTD(lossy_buffer);
      auto t3 = std::chrono::high_resolution_clock::now();
      auto new_size_lz4 = GetSizeLZ4(lossy_buffer);
      auto t4 = std::chrono::high_resolution_clock::now();

      stat.lossy.total_ratio += double(lossy_size) / double(ros_msg->data.size());
      stat.lossy.total_time += DurationUsec( (t2 - t1) );

      stat.lossy_zstd.total_ratio += double(new_size_zstd) / double(ros_msg->data.size());
      stat.lossy_zstd.total_time += DurationUsec( (t2 - t1) + (t3 - t2) );

      stat.lossy_lz4.total_ratio += double(new_size_lz4) / double(ros_msg->data.size());
      stat.lossy_lz4.total_time += DurationUsec( (t2 - t1) + (t4 - t3) );
    }
  }

  for (const auto& [topic, stat]: stats)    
  {
    double dcount = double(stat.count);
    std::cout << "\nTopic: " << topic << std::endl;
    std::cout << "  Count: " << stat.count << std::endl;
    printf("  [LZ4]  ratio: %.3f time (usec): %ld\n", stat.lz4.total_ratio / dcount, stat.lz4.total_time / stat.count);
    printf("  [ZSTD] ratio: %.3f time (usec): %ld\n", stat.zstd.total_ratio / dcount, stat.zstd.total_time / stat.count);

    printf("  [Lossy]        ratio: %.3f time (usec): %ld\n", stat.lossy.total_ratio / dcount, stat.lossy.total_time / stat.count);
    printf("  [Lossy + LZ4]  ratio: %.3f time (usec): %ld\n", stat.lossy_lz4.total_ratio / dcount, stat.lossy_lz4.total_time / stat.count);
    printf("  [Lossy + ZSTD] ratio: %.3f time (usec): %ld\n", stat.lossy_zstd.total_ratio / dcount, stat.lossy_zstd.total_time / stat.count);
  }

  return 0;
}
