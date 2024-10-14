#ifndef _ROS_ros_msd700_msgs_HardwareState_h
#define _ROS_ros_msd700_msgs_HardwareState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_msd700_msgs
{

  class HardwareState : public ros::Msg
  {
    public:
      typedef float _ch_ultrasonic_distance_1_type;
      _ch_ultrasonic_distance_1_type ch_ultrasonic_distance_1;
      typedef float _ch_ultrasonic_distance_2_type;
      _ch_ultrasonic_distance_2_type ch_ultrasonic_distance_2;
      typedef float _ch_ultrasonic_distance_3_type;
      _ch_ultrasonic_distance_3_type ch_ultrasonic_distance_3;
      typedef float _ch_ultrasonic_distance_4_type;
      _ch_ultrasonic_distance_4_type ch_ultrasonic_distance_4;
      typedef float _ch_ultrasonic_distance_5_type;
      _ch_ultrasonic_distance_5_type ch_ultrasonic_distance_5;
      typedef float _ch_ultrasonic_distance_6_type;
      _ch_ultrasonic_distance_6_type ch_ultrasonic_distance_6;
      typedef float _ch_ultrasonic_distance_7_type;
      _ch_ultrasonic_distance_7_type ch_ultrasonic_distance_7;
      typedef float _ch_ultrasonic_distance_8_type;
      _ch_ultrasonic_distance_8_type ch_ultrasonic_distance_8;
      typedef int32_t _right_motor_pulse_delta_type;
      _right_motor_pulse_delta_type right_motor_pulse_delta;
      typedef int32_t _left_motor_pulse_delta_type;
      _left_motor_pulse_delta_type left_motor_pulse_delta;
      typedef float _heading_type;
      _heading_type heading;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _acc_x_type;
      _acc_x_type acc_x;
      typedef float _acc_y_type;
      _acc_y_type acc_y;
      typedef float _acc_z_type;
      _acc_z_type acc_z;
      typedef float _gyr_x_type;
      _gyr_x_type gyr_x;
      typedef float _gyr_y_type;
      _gyr_y_type gyr_y;
      typedef float _gyr_z_type;
      _gyr_z_type gyr_z;
      typedef float _mag_x_type;
      _mag_x_type mag_x;
      typedef float _mag_y_type;
      _mag_y_type mag_y;
      typedef float _mag_z_type;
      _mag_z_type mag_z;

    HardwareState():
      ch_ultrasonic_distance_1(0),
      ch_ultrasonic_distance_2(0),
      ch_ultrasonic_distance_3(0),
      ch_ultrasonic_distance_4(0),
      ch_ultrasonic_distance_5(0),
      ch_ultrasonic_distance_6(0),
      ch_ultrasonic_distance_7(0),
      ch_ultrasonic_distance_8(0),
      right_motor_pulse_delta(0),
      left_motor_pulse_delta(0),
      heading(0),
      pitch(0),
      roll(0),
      acc_x(0),
      acc_y(0),
      acc_z(0),
      gyr_x(0),
      gyr_y(0),
      gyr_z(0),
      mag_x(0),
      mag_y(0),
      mag_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_1;
      u_ch_ultrasonic_distance_1.real = this->ch_ultrasonic_distance_1;
      *(outbuffer + offset + 0) = (u_ch_ultrasonic_distance_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch_ultrasonic_distance_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ch_ultrasonic_distance_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ch_ultrasonic_distance_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ch_ultrasonic_distance_1);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_2;
      u_ch_ultrasonic_distance_2.real = this->ch_ultrasonic_distance_2;
      *(outbuffer + offset + 0) = (u_ch_ultrasonic_distance_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch_ultrasonic_distance_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ch_ultrasonic_distance_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ch_ultrasonic_distance_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ch_ultrasonic_distance_2);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_3;
      u_ch_ultrasonic_distance_3.real = this->ch_ultrasonic_distance_3;
      *(outbuffer + offset + 0) = (u_ch_ultrasonic_distance_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch_ultrasonic_distance_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ch_ultrasonic_distance_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ch_ultrasonic_distance_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ch_ultrasonic_distance_3);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_4;
      u_ch_ultrasonic_distance_4.real = this->ch_ultrasonic_distance_4;
      *(outbuffer + offset + 0) = (u_ch_ultrasonic_distance_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch_ultrasonic_distance_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ch_ultrasonic_distance_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ch_ultrasonic_distance_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ch_ultrasonic_distance_4);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_5;
      u_ch_ultrasonic_distance_5.real = this->ch_ultrasonic_distance_5;
      *(outbuffer + offset + 0) = (u_ch_ultrasonic_distance_5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch_ultrasonic_distance_5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ch_ultrasonic_distance_5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ch_ultrasonic_distance_5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ch_ultrasonic_distance_5);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_6;
      u_ch_ultrasonic_distance_6.real = this->ch_ultrasonic_distance_6;
      *(outbuffer + offset + 0) = (u_ch_ultrasonic_distance_6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch_ultrasonic_distance_6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ch_ultrasonic_distance_6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ch_ultrasonic_distance_6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ch_ultrasonic_distance_6);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_7;
      u_ch_ultrasonic_distance_7.real = this->ch_ultrasonic_distance_7;
      *(outbuffer + offset + 0) = (u_ch_ultrasonic_distance_7.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch_ultrasonic_distance_7.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ch_ultrasonic_distance_7.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ch_ultrasonic_distance_7.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ch_ultrasonic_distance_7);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_8;
      u_ch_ultrasonic_distance_8.real = this->ch_ultrasonic_distance_8;
      *(outbuffer + offset + 0) = (u_ch_ultrasonic_distance_8.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ch_ultrasonic_distance_8.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ch_ultrasonic_distance_8.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ch_ultrasonic_distance_8.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ch_ultrasonic_distance_8);
      union {
        int32_t real;
        uint32_t base;
      } u_right_motor_pulse_delta;
      u_right_motor_pulse_delta.real = this->right_motor_pulse_delta;
      *(outbuffer + offset + 0) = (u_right_motor_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_left_motor_pulse_delta;
      u_left_motor_pulse_delta.real = this->left_motor_pulse_delta;
      *(outbuffer + offset + 0) = (u_left_motor_pulse_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_pulse_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_pulse_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_pulse_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor_pulse_delta);
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_acc_x;
      u_acc_x.real = this->acc_x;
      *(outbuffer + offset + 0) = (u_acc_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc_x);
      union {
        float real;
        uint32_t base;
      } u_acc_y;
      u_acc_y.real = this->acc_y;
      *(outbuffer + offset + 0) = (u_acc_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc_y);
      union {
        float real;
        uint32_t base;
      } u_acc_z;
      u_acc_z.real = this->acc_z;
      *(outbuffer + offset + 0) = (u_acc_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc_z);
      union {
        float real;
        uint32_t base;
      } u_gyr_x;
      u_gyr_x.real = this->gyr_x;
      *(outbuffer + offset + 0) = (u_gyr_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyr_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyr_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyr_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyr_x);
      union {
        float real;
        uint32_t base;
      } u_gyr_y;
      u_gyr_y.real = this->gyr_y;
      *(outbuffer + offset + 0) = (u_gyr_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyr_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyr_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyr_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyr_y);
      union {
        float real;
        uint32_t base;
      } u_gyr_z;
      u_gyr_z.real = this->gyr_z;
      *(outbuffer + offset + 0) = (u_gyr_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyr_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyr_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyr_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyr_z);
      union {
        float real;
        uint32_t base;
      } u_mag_x;
      u_mag_x.real = this->mag_x;
      *(outbuffer + offset + 0) = (u_mag_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mag_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mag_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mag_x);
      union {
        float real;
        uint32_t base;
      } u_mag_y;
      u_mag_y.real = this->mag_y;
      *(outbuffer + offset + 0) = (u_mag_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mag_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mag_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mag_y);
      union {
        float real;
        uint32_t base;
      } u_mag_z;
      u_mag_z.real = this->mag_z;
      *(outbuffer + offset + 0) = (u_mag_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mag_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mag_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mag_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_1;
      u_ch_ultrasonic_distance_1.base = 0;
      u_ch_ultrasonic_distance_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch_ultrasonic_distance_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ch_ultrasonic_distance_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ch_ultrasonic_distance_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ch_ultrasonic_distance_1 = u_ch_ultrasonic_distance_1.real;
      offset += sizeof(this->ch_ultrasonic_distance_1);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_2;
      u_ch_ultrasonic_distance_2.base = 0;
      u_ch_ultrasonic_distance_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch_ultrasonic_distance_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ch_ultrasonic_distance_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ch_ultrasonic_distance_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ch_ultrasonic_distance_2 = u_ch_ultrasonic_distance_2.real;
      offset += sizeof(this->ch_ultrasonic_distance_2);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_3;
      u_ch_ultrasonic_distance_3.base = 0;
      u_ch_ultrasonic_distance_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch_ultrasonic_distance_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ch_ultrasonic_distance_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ch_ultrasonic_distance_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ch_ultrasonic_distance_3 = u_ch_ultrasonic_distance_3.real;
      offset += sizeof(this->ch_ultrasonic_distance_3);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_4;
      u_ch_ultrasonic_distance_4.base = 0;
      u_ch_ultrasonic_distance_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch_ultrasonic_distance_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ch_ultrasonic_distance_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ch_ultrasonic_distance_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ch_ultrasonic_distance_4 = u_ch_ultrasonic_distance_4.real;
      offset += sizeof(this->ch_ultrasonic_distance_4);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_5;
      u_ch_ultrasonic_distance_5.base = 0;
      u_ch_ultrasonic_distance_5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch_ultrasonic_distance_5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ch_ultrasonic_distance_5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ch_ultrasonic_distance_5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ch_ultrasonic_distance_5 = u_ch_ultrasonic_distance_5.real;
      offset += sizeof(this->ch_ultrasonic_distance_5);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_6;
      u_ch_ultrasonic_distance_6.base = 0;
      u_ch_ultrasonic_distance_6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch_ultrasonic_distance_6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ch_ultrasonic_distance_6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ch_ultrasonic_distance_6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ch_ultrasonic_distance_6 = u_ch_ultrasonic_distance_6.real;
      offset += sizeof(this->ch_ultrasonic_distance_6);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_7;
      u_ch_ultrasonic_distance_7.base = 0;
      u_ch_ultrasonic_distance_7.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch_ultrasonic_distance_7.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ch_ultrasonic_distance_7.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ch_ultrasonic_distance_7.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ch_ultrasonic_distance_7 = u_ch_ultrasonic_distance_7.real;
      offset += sizeof(this->ch_ultrasonic_distance_7);
      union {
        float real;
        uint32_t base;
      } u_ch_ultrasonic_distance_8;
      u_ch_ultrasonic_distance_8.base = 0;
      u_ch_ultrasonic_distance_8.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ch_ultrasonic_distance_8.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ch_ultrasonic_distance_8.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ch_ultrasonic_distance_8.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ch_ultrasonic_distance_8 = u_ch_ultrasonic_distance_8.real;
      offset += sizeof(this->ch_ultrasonic_distance_8);
      union {
        int32_t real;
        uint32_t base;
      } u_right_motor_pulse_delta;
      u_right_motor_pulse_delta.base = 0;
      u_right_motor_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor_pulse_delta = u_right_motor_pulse_delta.real;
      offset += sizeof(this->right_motor_pulse_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_left_motor_pulse_delta;
      u_left_motor_pulse_delta.base = 0;
      u_left_motor_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor_pulse_delta = u_left_motor_pulse_delta.real;
      offset += sizeof(this->left_motor_pulse_delta);
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_acc_x;
      u_acc_x.base = 0;
      u_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc_x = u_acc_x.real;
      offset += sizeof(this->acc_x);
      union {
        float real;
        uint32_t base;
      } u_acc_y;
      u_acc_y.base = 0;
      u_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc_y = u_acc_y.real;
      offset += sizeof(this->acc_y);
      union {
        float real;
        uint32_t base;
      } u_acc_z;
      u_acc_z.base = 0;
      u_acc_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc_z = u_acc_z.real;
      offset += sizeof(this->acc_z);
      union {
        float real;
        uint32_t base;
      } u_gyr_x;
      u_gyr_x.base = 0;
      u_gyr_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyr_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyr_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyr_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyr_x = u_gyr_x.real;
      offset += sizeof(this->gyr_x);
      union {
        float real;
        uint32_t base;
      } u_gyr_y;
      u_gyr_y.base = 0;
      u_gyr_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyr_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyr_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyr_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyr_y = u_gyr_y.real;
      offset += sizeof(this->gyr_y);
      union {
        float real;
        uint32_t base;
      } u_gyr_z;
      u_gyr_z.base = 0;
      u_gyr_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyr_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyr_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyr_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyr_z = u_gyr_z.real;
      offset += sizeof(this->gyr_z);
      union {
        float real;
        uint32_t base;
      } u_mag_x;
      u_mag_x.base = 0;
      u_mag_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mag_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mag_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mag_x = u_mag_x.real;
      offset += sizeof(this->mag_x);
      union {
        float real;
        uint32_t base;
      } u_mag_y;
      u_mag_y.base = 0;
      u_mag_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mag_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mag_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mag_y = u_mag_y.real;
      offset += sizeof(this->mag_y);
      union {
        float real;
        uint32_t base;
      } u_mag_z;
      u_mag_z.base = 0;
      u_mag_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mag_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mag_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mag_z = u_mag_z.real;
      offset += sizeof(this->mag_z);
     return offset;
    }

    virtual const char * getType() override { return "ros_msd700_msgs/HardwareState"; };
    virtual const char * getMD5() override { return "83d55bb473af3c26f97964977e72f906"; };

  };

}
#endif
