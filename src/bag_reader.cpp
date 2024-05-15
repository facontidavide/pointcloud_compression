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

#include <draco/compression/expert_encode.h>
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>

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
  StatsData draco;
};

void ConvertFields(const sensor_msgs::msg::PointCloud2& msg, std::vector<Field>& fields, float resolution)
{
  fields.clear();
  for (const auto& field: msg.fields)
  {
    Field f;
    f.type = static_cast<FieldType>(field.datatype);
    f.offset = field.offset;
    // apply the resolution multiplier to position channels
    if((f.type == FieldType::FLOAT32 || f.type == FieldType::FLOAT64) &&
       (field.name == "x" || field.name == "y" || field.name == "z"))
    {
      f.mult = 1.0 / resolution;
    }
    // special field found in Hesai Lidar driver
    if(f.type == FieldType::FLOAT64 && field.name == "timestamp")
    {
      f.mult = 1e6; // convert to microseconds resolution
    }

    fields.push_back(f);
  }
}

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

// works only if one of the first 3 field is not finite
void RemoveInvalidPoints(const std::vector<uint8_t>& input, 
                         int point_step,
                         std::vector<uint8_t>& output)
{
  auto* ptr_end = input.data() + input.size();
  output.resize(input.size());

  auto* out_ptr = output.data();

  for(auto ptr = input.data();  ptr < ptr_end; ptr += point_step)
  {
    bool valid = true;
    for(size_t f=0; f<3; f++)
    {
      if(!std::isfinite(*reinterpret_cast<const float*>(ptr)))
      {
        valid = false;
        break;
      }
    }
    if(valid)
    {
      memcpy(out_ptr, ptr, point_step);
      out_ptr += point_step;
    }
  }
  output.resize( out_ptr - output.data() );
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
  std::vector<Field> fields;
  std::vector<uint8_t> clean_buffer;

  if(stats.empty())
  {
    std::cerr << "No PointCloud2 topics found in the bag" << std::endl;
    return 1;
  }

  while (rclcpp::ok() && reader.has_next()) {

    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

    if (stats.count(msg->topic_name) == 0) 
    {
      continue;
    }

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    serialization.deserialize_message(&serialized_msg, ros_msg.get());
   
    RemoveInvalidPoints(ros_msg->data, ros_msg->point_step, clean_buffer);

    auto& stat = stats[msg->topic_name];
    stat.count++;

    auto DurationUsec = [](const auto& diff) -> long
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
    };

    // straightforward ZSTD compression of ros_msg->data
    {
      auto t1 = std::chrono::high_resolution_clock::now(); 
      int new_size = GetSizeZSTD(clean_buffer);
      auto t2 = std::chrono::high_resolution_clock::now();

      stat.zstd.total_ratio += double(new_size) / double(ros_msg->data.size());
      stat.zstd.total_time += DurationUsec(t2 - t1);
    }
    // straightforward LZ4 compression of ros_msg->data
    {
      auto t1 = std::chrono::high_resolution_clock::now(); 
      int new_size = GetSizeLZ4(clean_buffer);
      auto t2 = std::chrono::high_resolution_clock::now();

      stat.lz4.total_ratio += double(new_size) / double(ros_msg->data.size());
      stat.lz4.total_time += DurationUsec(t2 - t1);
    }
    // // Lossy + compressions
    const float resolution = 0.001;
    {
      auto t1 = std::chrono::high_resolution_clock::now();
      ConvertFields(*ros_msg, fields, resolution);

      lossy_buffer.resize(clean_buffer.size());

      int lossy_size = PointcloudCompressionField(
        {fields.data(), fields.size()}, 
        ros_msg->point_step,
        {clean_buffer.data(), clean_buffer.size()}, 
        {lossy_buffer.data(), lossy_buffer.size()});

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

    // only PointXYZ and PointXYZI are supported here, to keep it simple
    if( ros_msg->point_step == 16 )
    {
      auto t1 = std::chrono::high_resolution_clock::now();
      draco::PointCloudBuilder builder;

      uint64_t number_of_points = clean_buffer.size() / ros_msg->point_step;
      builder.Start(static_cast<unsigned int>(number_of_points));

      auto* data_ptr = clean_buffer.data();

      for(const auto& field: ros_msg->fields)
      {
        if(field.name == "x" || field.name == "y" || field.name == "z")
        {
          int att_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 1, draco::DataType::DT_FLOAT32);
          builder.SetAttributeValuesForAllPoints(att_id, data_ptr + field.offset, ros_msg->point_step);
        }
        else
        {
          int att_id = builder.AddAttribute(draco::GeometryAttribute::GENERIC, 1, draco::DataType::DT_FLOAT32);
          builder.SetAttributeValuesForAllPoints(att_id, data_ptr + field.offset, ros_msg->point_step);
        }
      }

      std::unique_ptr<draco::PointCloud> pc = builder.Finalize(false);

      if (!pc) {
        throw std::runtime_error("Conversion from sensor_msgs::msg::PointCloud2 to Draco::PointCloud failed");
      }

      draco::Encoder encoder;
      encoder.SetSpeedOptions(0, 0);
      // Hard to decide if 16 bits is equivalent or not to a resolution of 1mm
      // this might affect the compression ratio and make the comparison unfair
      encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 16);
      encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 16);
      encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);     

      draco::EncoderBuffer encode_buffer;
      draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);

      uint32_t new_size = static_cast<uint32_t>(encode_buffer.size());
      auto t2 = std::chrono::high_resolution_clock::now();

      stat.draco.total_ratio += double(new_size) / double(ros_msg->data.size());
      stat.draco.total_time += DurationUsec(t2 - t1);
    }

  }


  for (const auto& [topic, stat]: stats)    
  {
    double dcount = double(stat.count);
    std::cout << "\nTopic: " << topic << std::endl;
    std::cout << "  Count: " << stat.count << std::endl;
    printf("  [LZ4]          ratio: %.2f time (usec): %ld\n", stat.lz4.total_ratio / dcount, stat.lz4.total_time / stat.count);
    printf("  [ZSTD]         ratio: %.2f time (usec): %ld\n", stat.zstd.total_ratio / dcount, stat.zstd.total_time / stat.count);
    printf("  [Draco]        ratio: %.2f time (usec): %ld\n", stat.draco.total_ratio / dcount, stat.draco.total_time / stat.count);
    printf("  [Lossy only]   ratio: %.2f time (usec): %ld\n", stat.lossy.total_ratio / dcount, stat.lossy.total_time / stat.count);
    printf("  [Lossy + LZ4]  ratio: %.2f time (usec): %ld\n", stat.lossy_lz4.total_ratio / dcount, stat.lossy_lz4.total_time / stat.count);
    printf("  [Lossy + ZSTD] ratio: %.2f time (usec): %ld\n", stat.lossy_zstd.total_ratio / dcount, stat.lossy_zstd.total_time / stat.count);


  }

  return 0;
}
