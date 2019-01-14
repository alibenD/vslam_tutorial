/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: test_config_yaml.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-14 14:15:18
  * @last_modified_date: 2019-01-14 15:23:31
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/config_yaml.hh>
#include <gtest/gtest.h>
#include <fstream>

//CODE
class ConfigYAMLTest : public testing::Test
{
  protected:
    virtual void SetUp() override
    {
      //config_yaml = std::make_shared<ak::ConfigYAML>("../config/default.yaml");
      config_yaml_ = std::make_shared<ak::ConfigYAML>();
      empty_file_path_ = "empty.yaml";
      std::ofstream outfile(empty_file_path_);
      outfile << "%YAML:1.0\n";
      outfile.close();
    }

    virtual void TearDown() override
    {
      std::remove(empty_file_path_.c_str());
    }

    ak::ConfigYAML::Ptr config_yaml_;
    std::string empty_file_path_;

};

TEST_F(ConfigYAMLTest, open)
{
  EXPECT_EQ(config_yaml_->open("empty.yaml"), 0);
}

TEST_F(ConfigYAMLTest, release)
{
  config_yaml_->open("empty.yaml");
  EXPECT_EQ(config_yaml_->release(), 0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
