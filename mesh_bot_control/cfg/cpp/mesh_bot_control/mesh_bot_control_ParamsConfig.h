//#line 2 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
// *********************************************************
// 
// File autogenerated for the mesh_bot_control package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

/***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************/

// Author: Blaise Gassend


#ifndef __mesh_bot_control__MESH_BOT_CONTROL_PARAMSCONFIG_H__
#define __mesh_bot_control__MESH_BOT_CONTROL_PARAMSCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/config_init_mutex.h>

namespace mesh_bot_control
{
  class mesh_bot_control_ParamsConfigStatics;
  
  class mesh_bot_control_ParamsConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(mesh_bot_control_ParamsConfig &config, const mesh_bot_control_ParamsConfig &max, const mesh_bot_control_ParamsConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const mesh_bot_control_ParamsConfig &config1, const mesh_bot_control_ParamsConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, mesh_bot_control_ParamsConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const mesh_bot_control_ParamsConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, mesh_bot_control_ParamsConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const mesh_bot_control_ParamsConfig &config) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T mesh_bot_control_ParamsConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (mesh_bot_control_ParamsConfig::* field);

      virtual void clamp(mesh_bot_control_ParamsConfig &config, const mesh_bot_control_ParamsConfig &max, const mesh_bot_control_ParamsConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const mesh_bot_control_ParamsConfig &config1, const mesh_bot_control_ParamsConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, mesh_bot_control_ParamsConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const mesh_bot_control_ParamsConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, mesh_bot_control_ParamsConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const mesh_bot_control_ParamsConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }
    };

//#line 13 "../cfg/mesh_bot_control.cfg"
      double base_linear_speed;
//#line 14 "../cfg/mesh_bot_control.cfg"
      double turbo_linear;
//#line 15 "../cfg/mesh_bot_control.cfg"
      double base_rot_speed;
//#line 16 "../cfg/mesh_bot_control.cfg"
      double turbo_angular;
//#line 17 "../cfg/mesh_bot_control.cfg"
      double d_pad_percent;
//#line 18 "../cfg/mesh_bot_control.cfg"
      double kinect_reset_ang;
//#line 19 "../cfg/mesh_bot_control.cfg"
      std::string plus_mesage;
//#line 20 "../cfg/mesh_bot_control.cfg"
      std::string minus_sound;
//#line 21 "../cfg/mesh_bot_control.cfg"
      std::string waypoint_sound;
//#line 22 "../cfg/mesh_bot_control.cfg"
      std::string done_sound;
//#line 138 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        if ((*i)->fromMessage(msg, *this))
          count++;
      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("mesh_bot_control_ParamsConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toMessage(msg, *this);
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      __toMessage__(msg, __param_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->fromServer(nh, *this);
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const mesh_bot_control_ParamsConfig &__max__ = __getMax__();
      const mesh_bot_control_ParamsConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const mesh_bot_control_ParamsConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const mesh_bot_control_ParamsConfig &__getDefault__();
    static const mesh_bot_control_ParamsConfig &__getMax__();
    static const mesh_bot_control_ParamsConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    
  private:
    static const mesh_bot_control_ParamsConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void mesh_bot_control_ParamsConfig::ParamDescription<std::string>::clamp(mesh_bot_control_ParamsConfig &config, const mesh_bot_control_ParamsConfig &max, const mesh_bot_control_ParamsConfig &min) const
  {
    return;
  }

  class mesh_bot_control_ParamsConfigStatics
  {
    friend class mesh_bot_control_ParamsConfig;
    
    mesh_bot_control_ParamsConfigStatics()
    {
//#line 13 "../cfg/mesh_bot_control.cfg"
      __min__.base_linear_speed = 0.0;
//#line 13 "../cfg/mesh_bot_control.cfg"
      __max__.base_linear_speed = 10.0;
//#line 13 "../cfg/mesh_bot_control.cfg"
      __default__.base_linear_speed = 0.05;
//#line 13 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<double>("base_linear_speed", "double", 0, "max robot speed in lineardirection", "", &mesh_bot_control_ParamsConfig::base_linear_speed)));
//#line 14 "../cfg/mesh_bot_control.cfg"
      __min__.turbo_linear = 0.0;
//#line 14 "../cfg/mesh_bot_control.cfg"
      __max__.turbo_linear = 5.0;
//#line 14 "../cfg/mesh_bot_control.cfg"
      __default__.turbo_linear = 0.1;
//#line 14 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<double>("turbo_linear", "double", 0, "multiplyer of turbo button y", "", &mesh_bot_control_ParamsConfig::turbo_linear)));
//#line 15 "../cfg/mesh_bot_control.cfg"
      __min__.base_rot_speed = 0.0;
//#line 15 "../cfg/mesh_bot_control.cfg"
      __max__.base_rot_speed = 4.0;
//#line 15 "../cfg/mesh_bot_control.cfg"
      __default__.base_rot_speed = 0.3;
//#line 15 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<double>("base_rot_speed", "double", 0, "max robot speed in angular direction", "", &mesh_bot_control_ParamsConfig::base_rot_speed)));
//#line 16 "../cfg/mesh_bot_control.cfg"
      __min__.turbo_angular = 0.0;
//#line 16 "../cfg/mesh_bot_control.cfg"
      __max__.turbo_angular = 5.0;
//#line 16 "../cfg/mesh_bot_control.cfg"
      __default__.turbo_angular = 0.6;
//#line 16 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<double>("turbo_angular", "double", 0, "multiplyer of turbo button x", "", &mesh_bot_control_ParamsConfig::turbo_angular)));
//#line 17 "../cfg/mesh_bot_control.cfg"
      __min__.d_pad_percent = 0.0;
//#line 17 "../cfg/mesh_bot_control.cfg"
      __max__.d_pad_percent = 100.0;
//#line 17 "../cfg/mesh_bot_control.cfg"
      __default__.d_pad_percent = 50.0;
//#line 17 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<double>("d_pad_percent", "double", 0, "the percent that the dpad is scaled down", "", &mesh_bot_control_ParamsConfig::d_pad_percent)));
//#line 18 "../cfg/mesh_bot_control.cfg"
      __min__.kinect_reset_ang = -30.0;
//#line 18 "../cfg/mesh_bot_control.cfg"
      __max__.kinect_reset_ang = 30.0;
//#line 18 "../cfg/mesh_bot_control.cfg"
      __default__.kinect_reset_ang = 0.0;
//#line 18 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<double>("kinect_reset_ang", "double", 0, "angle to reset the kinect to", "", &mesh_bot_control_ParamsConfig::kinect_reset_ang)));
//#line 19 "../cfg/mesh_bot_control.cfg"
      __min__.plus_mesage = "";
//#line 19 "../cfg/mesh_bot_control.cfg"
      __max__.plus_mesage = "";
//#line 19 "../cfg/mesh_bot_control.cfg"
      __default__.plus_mesage = "Pardon me. coming through.";
//#line 19 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<std::string>("plus_mesage", "str", 0, "phrase to say when plus button is pressed", "", &mesh_bot_control_ParamsConfig::plus_mesage)));
//#line 20 "../cfg/mesh_bot_control.cfg"
      __min__.minus_sound = "";
//#line 20 "../cfg/mesh_bot_control.cfg"
      __max__.minus_sound = "";
//#line 20 "../cfg/mesh_bot_control.cfg"
      __default__.minus_sound = "/home/robot/Documents/Joemegatron_IGVC_2011/sounds/Ahooga_Car_Horn.wav";
//#line 20 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<std::string>("minus_sound", "str", 0, "sound to play when minus button is pressed", "", &mesh_bot_control_ParamsConfig::minus_sound)));
//#line 21 "../cfg/mesh_bot_control.cfg"
      __min__.waypoint_sound = "";
//#line 21 "../cfg/mesh_bot_control.cfg"
      __max__.waypoint_sound = "";
//#line 21 "../cfg/mesh_bot_control.cfg"
      __default__.waypoint_sound = "/home/robot/Documents/Joemegatron_IGVC_2011/sounds/Ahooga_Car_Horn.wav";
//#line 21 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<std::string>("waypoint_sound", "str", 0, "sound to play when minus button is pressed", "", &mesh_bot_control_ParamsConfig::waypoint_sound)));
//#line 22 "../cfg/mesh_bot_control.cfg"
      __min__.done_sound = "";
//#line 22 "../cfg/mesh_bot_control.cfg"
      __max__.done_sound = "";
//#line 22 "../cfg/mesh_bot_control.cfg"
      __default__.done_sound = "/home/robot/Documents/Joemegatron_IGVC_2011/sounds/Ahooga_Car_Horn.wav";
//#line 22 "../cfg/mesh_bot_control.cfg"
      __param_descriptions__.push_back(mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr(new mesh_bot_control_ParamsConfig::ParamDescription<std::string>("done_sound", "str", 0, "sound to play when minus button is pressed", "", &mesh_bot_control_ParamsConfig::done_sound)));
//#line 239 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
    
      for (std::vector<mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        __description_message__.parameters.push_back(**i);
      __max__.__toMessage__(__description_message__.max, __param_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__); 
    }
    std::vector<mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    mesh_bot_control_ParamsConfig __max__;
    mesh_bot_control_ParamsConfig __min__;
    mesh_bot_control_ParamsConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;
    static const mesh_bot_control_ParamsConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static mesh_bot_control_ParamsConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &mesh_bot_control_ParamsConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const mesh_bot_control_ParamsConfig &mesh_bot_control_ParamsConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const mesh_bot_control_ParamsConfig &mesh_bot_control_ParamsConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const mesh_bot_control_ParamsConfig &mesh_bot_control_ParamsConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<mesh_bot_control_ParamsConfig::AbstractParamDescriptionConstPtr> &mesh_bot_control_ParamsConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const mesh_bot_control_ParamsConfigStatics *mesh_bot_control_ParamsConfig::__get_statics__()
  {
    const static mesh_bot_control_ParamsConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = mesh_bot_control_ParamsConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __MESH_BOT_CONTROL_PARAMSRECONFIGURATOR_H__
