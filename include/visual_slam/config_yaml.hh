#ifndef __CONFIG_YAML_HH__
#define __CONFIG_YAML_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: config_yaml.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-14 09:39:12
  * @last_modified_date: 2019-01-14 14:02:00
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam/config_base.hh>
#include <memory>
#include <opencv2/core.hpp>

// Declaration
namespace ak
{
  class ConfigYAML : public ConfigBase, public std::enable_shared_from_this<ConfigYAML>
  {
    public:
      using Ptr = std::shared_ptr<ConfigYAML>;
      ConfigYAML() = default;
      ConfigYAML(const std::string& path);
      virtual ~ConfigYAML();
      static Ptr ptr_config_;

    public:
      virtual int open(const std::string& path) override;
      virtual int release() override;
      int useSharedThis()
      {
        ConfigYAML::ptr_config_ = this->shared_from_this();
        //shared_from_this();
        return 0;
      }

      template <typename T>
        T getItem(const std::string& item_name)
        {
          return T( (*ptr_yaml_handler_)[item_name] );
        }

    private:
      cv::FileStorage* ptr_yaml_handler_;
  };
}
#endif // __CONFIG_YAML_HH__
