#!/usr/bin/env python


from dynamic_reconfigure.parameter_generator_catkin import *

def add_base_params(gen, prefix):
    '''
    Adds the General D400-series parameters to the given 
    dynamic_reconfigure ParameterGenerator.

    The level value of the parameters will start at value 1 and
    increase by 1 for each parameter added. The next level value
    in the sequence will be returned. The caller should use this
    returned value as the first level value for any device-
    specific parameters.

    :param gen: The ParameterGenerator to add property definitions to.
    :param prefix:  The prefix that will be prepended to each property name.
    :return: The next value in the level sequence, to be used as the next
    level value when adding device-specific properties.
    '''

    prefix = str(prefix)

    preset_enum = gen.enum([gen.const("Custom",        int_t,  0,  "Custom"),
                            gen.const("Default",       int_t,  1,  "Default Preset"),
                            gen.const("Hand",          int_t,  2,  "Hand Gesture"),
                            gen.const("HighAccuracy",  int_t,  3,  "High Accuracy"),
                            gen.const("HighDensity",   int_t,  4,  "High Density"),
                            gen.const("MediumDensity", int_t,  5,  "Medium Density")], "D400 Visual Presets")

    spatial_filter_holes_fill_enum = gen.enum([gen.const("SpatialFillHoleDisabled", int_t,  0,  "Spatial - Fill Hole Disabled"),
                                               gen.const("2PxielRadius",            int_t,  1,  "2-Pixel Radius"),
                                               gen.const("4PxielRadius",            int_t,  2,  "4-Pixel Radius"),
                                               gen.const("8PxielRadius",            int_t,  3,  "8-Pixel Radius"),
                                               gen.const("16PxielRadius",           int_t,  4,  "16-Pixel Radius"),
                                               gen.const("Unlimited",               int_t,  5,  "Unlimited")], "Spatial Filter Holes Fill")

    temporal_filter_holes_fill_enum = gen.enum([gen.const("TemporalFillHoleDisabled",        int_t,  0,  "Temporal - Fill Hole Disabled"),
                                                gen.const("ValidIn8Of8",                     int_t,  1,  "Valid In 8 Of 8"),
                                                gen.const("ValidIn2Oflast3",                 int_t,  2,  "Valid In 2 Of last3"),
                                                gen.const("ValidIn2Oflast4",                 int_t,  3,  "Valid In 2 Of last4"),
                                                gen.const("ValidIn2Of8",                     int_t,  4,  "Valid In 2 Of 8"),
                                                gen.const("ValidIn1Oflast2",                 int_t,  5,  "Valid In 1 Of last2"),
                                                gen.const("ValidIn1Oflast5",                 int_t,  6,  "Valid in 1 Of last5")], "Temporal Filter Holes Fill")

    level = [1] # we have to store this counter as a list since python2 doesn't like rebinding captured variables.
    def add(name, datatype, description, default, minval=None, maxval=None, **kwargs):
        gen.add(name, datatype, level[0], description, default, minval, maxval, **kwargs)
        level[0] += 1

    # These should be in the same order as the enum values of base_depth_param
    #             Name                                          Type     Description                  Default    Min     Max
    add(prefix + "depth_gain",                              int_t,    "Gain",                             16,    16,     248)
    add(prefix + "depth_enable_auto_exposure",              bool_t,   "Enable Auto Exposure",             False)
    add(prefix + "depth_visual_preset",                     int_t,    "D400 Visual Presets",              0,      0, 5, edit_method=preset_enum)
    add(prefix + "depth_frames_queue_size",                 int_t,    "Frames Queue Size",                16,     0,      32)
    add(prefix + "depth_error_polling_enabled",             bool_t,   "Error Polling Enabled",            False)
    add(prefix + "depth_output_trigger_enabled",            bool_t,   "Output Trigger Enabled",           False)
    add(prefix + "depth_units",                             double_t, "Depth Units",                      0.001,  0.001,  0.001)
    add(prefix + "JSON_file_path",                          str_t,    "JSON_file_path",                   "")
    add(prefix + "enable_depth_to_disparity_filter",        bool_t,   "Enable Depth to Disparity Filter", False)
    add(prefix + "enable_spatial_filter",                   bool_t,   "Enable Spatial Filter",            False)
    add(prefix + "enable_temporal_filter",                  bool_t,   "Enable Temporal Filter",           False)
    add(prefix + "enable_disparity_to_depth_filter",        bool_t,   "Enable Disparity to Depth Filter", False)
    add(prefix + "spatial_filter_magnitude",                double_t, "Spatial Filter Magnitude",         2.0,    1.0,   5.0)
    add(prefix + "spatial_filter_smooth_alpha",             double_t, "Spatial Filter Smooth Alpha",      0.5,    0.25,  1.0)
    add(prefix + "spatial_filter_smooth_delta",             double_t, "Spatial Filter Smooth Delta",      20.0,   1.0,   50.0)
    add(prefix + "spatial_filter_holes_fill",               int_t,    "Spatial Filter Holes Filter",      0,      0,     5, edit_method=spatial_filter_holes_fill_enum)
    add(prefix + "temporal_filter_smooth_alpha",            double_t, "Temporal Smooth Alpha",            0.4,    0.0,   1.0)
    add(prefix + "temporal_filter_smooth_delta",            double_t, "Temporal Smooth Delta",            20.0,   1.0,   100.0)
    add(prefix + "temporal_filter_holes_fill",              int_t,    "Temporal Filter Holes Fill",       3,      0,     6, edit_method=temporal_filter_holes_fill_enum)

    return level[0]
