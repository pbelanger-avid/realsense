#include <realsense2_camera/rs415_param_manager.h>
#include <realsense2_camera/rs415_paramsConfig.h>

using namespace realsense2_camera;

void RS415ParamManager::registerDynamicReconfigCb(RealSenseNode *node_ptr)
{
    _server = std::make_shared<dynamic_reconfigure::Server<rs415_paramsConfig>>();
    _f = boost::bind(&RS415ParamManager::callback, this, node_ptr, _1, _2);
    _server->setCallback(_f);
}


void RS415ParamManager::setParam(RealSenseNode* node_ptr, rs415_paramsConfig &config, rs415_param param)
{
    // W/O for zero param
    if (0 == param)
        return;

    switch (param) {
    case rs415_color_backlight_compensation:
        ROS_DEBUG_STREAM("base_JSON_file_path: " << config.rs415_color_backlight_compensation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_BACKLIGHT_COMPENSATION, config.rs415_color_backlight_compensation);
        break;
    case rs415_color_brightness:
        ROS_DEBUG_STREAM("rs415_color_brightness: " << config.rs415_color_brightness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_BRIGHTNESS, config.rs415_color_brightness);
        break;
    case rs415_color_contrast:
        ROS_DEBUG_STREAM("rs415_color_contrast: " << config.rs415_color_contrast);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_CONTRAST, config.rs415_color_contrast);
        break;
    case rs415_color_gain:
        ROS_DEBUG_STREAM("rs415_color_gain: " << config.rs415_color_gain);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_GAIN, config.rs415_color_gain);
        break;
    case rs415_color_gamma:
        ROS_DEBUG_STREAM("rs415_color_gamma: " << config.rs415_color_gamma);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_GAMMA, config.rs415_color_gamma);
        break;
    case rs415_color_hue:
        ROS_DEBUG_STREAM("rs415_color_hue: " << config.rs415_color_hue);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_HUE, config.rs415_color_hue);
        break;
    case rs415_color_saturation:
        ROS_DEBUG_STREAM("rs415_color_saturation: " << config.rs415_color_saturation);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_SATURATION, config.rs415_color_saturation);
        break;
    case rs415_color_sharpness:
        ROS_DEBUG_STREAM("rs415_color_sharpness: " << config.rs415_color_sharpness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_SHARPNESS, config.rs415_color_sharpness);
        break;
    case rs415_color_enable_auto_white_balance:
        ROS_DEBUG_STREAM("rs415_color_sharpness: " << config.rs415_color_sharpness);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.rs415_color_enable_auto_white_balance);
        break;
    case rs415_color_enable_auto_exposure:
        ROS_DEBUG_STREAM("rs415_color_sharpness: " << config.rs415_color_enable_auto_exposure);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, config.rs415_color_enable_auto_exposure);
        break;
    case rs415_color_exposure:
        ROS_DEBUG_STREAM("rs415_color_exposure: " << config.rs415_color_exposure);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.rs415_color_exposure);
        break;
    case rs415_color_white_balance:
    {
        static const auto rs415_color_white_balance_factor = 10;
        ROS_DEBUG_STREAM("rs415_color_white_balance: " << config.rs415_color_white_balance * rs415_color_white_balance_factor);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_WHITE_BALANCE, config.rs415_color_white_balance * rs415_color_white_balance_factor);
    }
        break;
    case rs415_color_frames_queue_size:
        ROS_DEBUG_STREAM("rs415_color_frames_queue_size: " << config.rs415_color_frames_queue_size);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_FRAMES_QUEUE_SIZE, config.rs415_color_frames_queue_size);
        break;
    case rs415_color_power_line_frequency:
        ROS_DEBUG_STREAM("rs415_color_power_line_frequency: " << config.rs415_color_power_line_frequency);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_POWER_LINE_FREQUENCY, config.rs415_color_power_line_frequency);
        break;
    case rs415_color_auto_exposure_priority:
        ROS_DEBUG_STREAM("rs415_color_power_line_frequency: " << config.rs415_color_auto_exposure_priority);
        node_ptr->_sensors[RealSenseNode::COLOR].set_option(rs2_option::RS2_OPTION_AUTO_EXPOSURE_PRIORITY, config.rs415_color_auto_exposure_priority);
        break;
    case rs415_depth_exposure:
    {
        static const auto rs415_depth_exposure_factor = 20;
        ROS_DEBUG_STREAM("rs415_depth_exposure: " << config.rs415_depth_exposure * rs415_depth_exposure_factor);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_EXPOSURE, config.rs415_depth_exposure * rs415_depth_exposure_factor);
    }
        break;
    case rs415_depth_laser_power:
    {
        static const auto rs415_depth_laser_power_factor = 30;
        ROS_DEBUG_STREAM("rs415_depth_laser_power: " << config.rs415_depth_laser_power * rs415_depth_laser_power_factor);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_LASER_POWER, config.rs415_depth_laser_power * rs415_depth_laser_power_factor);
    }
        break;
    case rs415_depth_emitter_enabled:
        ROS_DEBUG_STREAM("rs415_depth_emitter_enabled: " << config.rs415_depth_emitter_enabled);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, config.rs415_depth_emitter_enabled);
        break;
        case rs415_depth_enable_auto_white_balance:
        ROS_DEBUG_STREAM("rs415_depth_enable_auto_white_balance: " << config.rs415_depth_enable_auto_white_balance);
        node_ptr->_sensors[RealSenseNode::DEPTH].set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, config.rs415_depth_enable_auto_white_balance);
        break;
    default:
        //D400ParamManager::setParam(config, (base_depth_param)param);
        break;
    }
}

void RS415ParamManager::callback(RealSenseNode* node_ptr, rs415_paramsConfig &config, uint32_t level)
{
    ROS_DEBUG_STREAM("RS415ParamManager - Level: " << level);

    if (node_ptr->set_default_dynamic_reconfig_values == level)
    {
        for (int i = 1 ; i < rs415_param_count ; ++i)
        {
            ROS_DEBUG_STREAM("rs415_param = " << i);
            setParam(node_ptr, config ,(rs415_param)i);
        }
    }
    else
    {
        setParam(node_ptr, config, (rs415_param)level);
    }
}
