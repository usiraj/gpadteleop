## *********************************************************
## 
## File autogenerated for the gpadteleop package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'Maximum Linear Velcoity of Robot', 'max': 30.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_lin_vel', 'edit_method': '', 'default': 1.0, 'level': 16384, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Maximum Angular Velocity of Robot', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_ang_vel', 'edit_method': '', 'default': 0.5, 'level': 32768, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Slow Mode Linear Speed of Robot in Percentage of Maximum Speed', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'percent_lin_slow', 'edit_method': '', 'default': 0.2, 'level': 16384, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Medium Mode Linear Speed of Robot in Percentage of Maximum Speed', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'percent_lin_medium', 'edit_method': '', 'default': 0.6, 'level': 16384, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Fast Mode Linear Speed of Robot in Percentage of Maximum Speed', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'percent_lin_fast', 'edit_method': '', 'default': 1.0, 'level': 16384, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Percentage Increment of Linear Speed in Fixed Speed Mode', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'percent_lin_inc', 'edit_method': '', 'default': 0.1, 'level': 16384, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Percentage Increment of Angular Speed in Fixed Speed Mode', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'percent_ang_inc', 'edit_method': '', 'default': 0.1, 'level': 32768, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Slow Mode Angular Speed of Robot in Percentage of Maximum Speed', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'percent_ang_slow', 'edit_method': '', 'default': 0.2, 'level': 32768, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Medium Mode Angular Speed of Robot in Percentage of Maximum Speed', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'percent_ang_medium', 'edit_method': '', 'default': 0.6, 'level': 32768, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Fast Mode Angular Speed of Robot in Percentage of Maximum Speed', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'percent_ang_fast', 'edit_method': '', 'default': 1.0, 'level': 32768, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'GamePad Axis ID for Linear Motion', 'max': 6, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'axis_linear', 'edit_method': '', 'default': 1, 'level': 1024, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Axis ID for Angular Motion', 'max': 6, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'axis_angular', 'edit_method': '', 'default': 3, 'level': 4096, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Axis ID for Linear Motion Speed', 'max': 6, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'axis_linear_speed', 'edit_method': '', 'default': 6, 'level': 2048, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Axis ID for Angular Motion Speed', 'max': 6, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'axis_angular_speed', 'edit_method': '', 'default': 5, 'level': 8192, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Button ID for Linear Speed Mode Selection', 'max': 11, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'btn_lin_mode', 'edit_method': '', 'default': 6, 'level': 256, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Button ID for Angular Speed Mode Selection', 'max': 11, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'btn_ang_mode', 'edit_method': '', 'default': 7, 'level': 512, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Button ID for Hard Brake which also disables motors', 'max': 11, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'btn_hard_brake', 'edit_method': '', 'default': 4, 'level': 128, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Button ID for Soft Brake', 'max': 11, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'btn_soft_brake', 'edit_method': '', 'default': 5, 'level': 64, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Button ID for engaging/disengaging of lock', 'max': 11, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'btn_robot_lock', 'edit_method': '', 'default': 3, 'level': 32, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Button ID for toggling Speed Mode of Linear Motion', 'max': 11, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'btn_speed_mode_lin', 'edit_method': '', 'default': 10, 'level': 8, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'GamePad Button ID for toggling Speed Mode of Angular Motion', 'max': 11, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'btn_speed_mode_ang', 'edit_method': '', 'default': 11, 'level': 16, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Publish Frequency of Commands in Speed Mode in Hz', 'max': 120, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'publish_freq', 'edit_method': '', 'default': 30, 'level': 4, 'min': 1, 'type': 'int'}, {'srcline': 259, 'description': 'Service to Enable Motors', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'srv_motors_enable', 'edit_method': '', 'default': 'enable_motors', 'level': 1, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Service to Disable Motors', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'srv_motors_disable', 'edit_method': '', 'default': 'disable_motors', 'level': 2, 'min': '', 'type': 'str'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

