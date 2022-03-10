/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Sanjay Jaiman <sjaiman@parallelwireless,com>
 */
#ifndef UE_CONFIG_H
#define UE_CONFIG_H

#include <iostream>
#include <vector>
#include <memory>
#include "ns3/object.h"
#include "ns3/vector.h"
#include "ns3/node-container.h"
#include <ns3/mobility-building-info.h>
#include <ns3/constant-position-mobility-model.h>
#include "ns3/mobility-module.h"

#define CONST_POS_STR "const_position"
#define CONST_POS_IDLE_STR "const_position_idle"
#define RAND_WALK_STR "rand_walk"
#define RAND_WALK_IDLE_STR "rand_walk_idle"
#define CONST_VELOCITY_STR "const_velocity"
#define CONST_VELOCITY_IDLE_STR "const_velocity_idle"

#define CONST_POS_ACTIVE_MASK 0x00000001
#define CONST_POS_IDLE_MASK 0x00000002
#define RAND_WALK_ACTIVE_MASK 0x00000004
#define RAND_WALK_IDLE_MASK 0x00000008
#define CONST_VELOCITY_ACTIVE_MASK 0x00000010
#define CONST_VELOCITY_IDLE_MASK 0x00000020

namespace ns3 {

class UeDesc : public Object {
public:
//  UeDesc() {};
  UeDesc(uint32_t id_, Vector pos_, bool active, uint32_t mask) : id(id_), model_type(mask), pos(pos_), active_(active) {};

public:
  bool is_active() {return active_;};
  virtual std::string toString() = 0;
  virtual Vector get_constant_vel_speed() = 0;
  virtual std::vector<double> get_rand_walk_rect() = 0;
public:
  uint32_t id;
  uint32_t enb_id;
  uint32_t NodeId;
  uint32_t model_type;
  Vector pos;
  bool   active_;
public:
  std::string to_string() {
    std::stringstream str;
    str << id << " (" << pos.x << "," << pos.y << "," << pos.z << ") ; Type = " << 
      ((model_type == CONST_POS_ACTIVE_MASK || model_type == CONST_POS_IDLE_MASK ) ? "Constant Position " :
        ((model_type == RAND_WALK_ACTIVE_MASK || model_type == RAND_WALK_IDLE_MASK) ? "Random Walk " :
          ((model_type == CONST_VELOCITY_ACTIVE_MASK || model_type == CONST_VELOCITY_IDLE_MASK) ? "Constant Velocity " : "N/A")))
    << ((is_active()) ? ";Active" : ";Idle");
//    str << "; ENB_ID = " << enb_id;
    return str.str();
  };
  void set_enb_id(uint32_t id) {enb_id = id;};
};

class UeDescConstantPos :  public UeDesc {
public:
  UeDescConstantPos(uint32_t id_, Vector pos_, bool active, uint32_t mask) : UeDesc(id_, pos_, active, mask) {};
public:
  virtual std::string toString()  {
    std::ostringstream os;
    os << "cfg [" << id << "] = " << to_string();
    return os.str();
  };
  virtual Vector get_constant_vel_speed() {NS_ASSERT_MSG(true, "NO Velocity defined"); return Vector();};
  virtual std::vector<double> get_rand_walk_rect() {NS_ASSERT_MSG(true, "NO rand walk rectangle defined"); return std::vector<double>();};
};

class UeDescConstantPosActive :  public UeDescConstantPos {
public:
  UeDescConstantPosActive(uint32_t id_, Vector pos_);
};
class UeDescConstantPosIdle :  public UeDescConstantPos {
public:
  UeDescConstantPosIdle(uint32_t id_, Vector pos_);
};

class UeDescRandWalk :  public UeDesc {
public:
  UeDescRandWalk(uint32_t id_, Vector pos_, bool active, std::vector<double> walk_rect_, uint32_t mask);

public:
  virtual std::string toString()  {
    std::ostringstream os;
    os << "cfg [" << id << "] = " << to_string();
    os << "; Rect= (" << walk_rect[0] << ", " << walk_rect[1] << ", " << walk_rect[2] << ", " << walk_rect[3] << ")";
    os << "]";
    return os.str();
  };
  virtual std::vector<double> get_rand_walk_rect() {return walk_rect;};
  virtual Vector get_constant_vel_speed()  {NS_ASSERT_MSG(true, "NO Velocity defined"); return Vector();};
private:
  std::vector<double> walk_rect;
};

class UeDescRandWalkActive :  public UeDescRandWalk {
public:
  UeDescRandWalkActive(uint32_t id_, Vector pos_, std::vector<double> walk_rect_);
};

class UeDescRandWalkIdle :  public UeDescRandWalk {
public:
  UeDescRandWalkIdle(uint32_t id_, Vector pos_, std::vector<double> walk_rect_);
};

class UeDescConstantVel :  public UeDesc {
public:
  UeDescConstantVel(uint32_t id_, Vector pos_, bool active, Vector speed_, uint32_t mask);

public:
  virtual std::string toString()  {
    std::ostringstream os;
    os << "cfg [" << id << "] = " << to_string();
    os << "; Velocity = (" << speed.x << ", " << speed.y << ", " << speed.z << ")";
    os << "]";
    return os.str();
  };
  virtual Vector get_constant_vel_speed() {return speed;}
  virtual std::vector<double> get_rand_walk_rect()  {NS_ASSERT_MSG(true, "NO rand walk rectangle defined"); return std::vector<double>();};
private:
  Vector speed;
};

class UeDescConstantVelActive :  public UeDescConstantVel {
public:
  UeDescConstantVelActive(uint32_t id_, Vector pos_, Vector speed);
};

class UeDescConstantVelIdle :  public UeDescConstantVel {
public:
  UeDescConstantVelIdle(uint32_t id_, Vector pos_, Vector speed);
};

class UeConfig : public Object
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  UeConfig ();
  UeConfig (std::string ue_pos_file);
  void init();
  virtual ~UeConfig () {};

  /**
   * \brief Add the positions listed in a file.
   * The file should be a simple text file, with one position per line,
   * either X and Y, or X, Y and Z, in meters.  The delimiter can
   * be any character, such as ',' or '\t'; the default is a comma ','.
   *
   * The file is read using CsvReader, which explains how comments
   * and whitespace are handled.
   *
   * \param [in] filePath The path to the input file.
   * \param [in] delimiter The delimiter character; see CsvReader.
   */
  void add_ues (const std::string filePath, char delimiter = ',');
  void add_active_ue (double x, double y, double z);

  uint32_t GetSize (void) const;

  Ptr<ListPositionAllocator> make_list_position(Ptr<ListPositionAllocator>& list_, uint32_t mask);
  Ptr<ListPositionAllocator> get_const_position_active_alloc() {return cp_active_list_position;}
  Ptr<ListPositionAllocator> get_const_position_idle_alloc() {return cp_idle_list_position;}
  Ptr<ListPositionAllocator> get_rand_walk_active_alloc() {return rw_active_list_position;}
  Ptr<ListPositionAllocator> get_rand_walk_idle_alloc() {return rw_idle_list_position;}
  Ptr<ListPositionAllocator> get_const_velocity_active_alloc() {return cv_active_list_position;}
  Ptr<ListPositionAllocator> get_const_velocity_idle_alloc() {return cv_idle_list_position;}

  uint32_t get_num_const_pos_active_ues() {return cp_active_list_position->GetSize();};
  uint32_t get_num_const_pos_idle_ues() {return cp_idle_list_position->GetSize();};
  uint32_t get_num_rand_walk_active_ues() {return rw_active_list_position->GetSize();};
  uint32_t get_num_rand_walk_idle_ues() {return rw_idle_list_position->GetSize();};
  uint32_t get_num_const_vel_active_ues() {return cv_active_list_position->GetSize();};
  uint32_t get_num_const_vel_idle_ues() {return cv_idle_list_position->GetSize();};

  std::vector<Ptr<UeDesc>>& get_active_ue_desc_vector() {return ue_vector_active;};
  std::vector<Ptr<UeDesc>>& get_idle_ue_desc_vector() {return ue_vector_idle;};
  uint32_t get_num_idle_nodes() {return ue_vector_idle.size();}
  void print (std::ostringstream& os);

private:
  std::vector<Ptr<UeDesc>> ue_vector_active;
  std::vector<Ptr<UeDesc>> ue_vector_idle;
  bool get_ue_struct(Ptr<UeDesc>& ret, uint32_t index);

public:
  Ptr<ListPositionAllocator> cp_active_list_position;
  Ptr<ListPositionAllocator> cp_idle_list_position;
  Ptr<ListPositionAllocator> rw_active_list_position;
  Ptr<ListPositionAllocator> rw_idle_list_position;
  Ptr<ListPositionAllocator> cv_active_list_position;
  Ptr<ListPositionAllocator> cv_idle_list_position;
};


} // namespace ns3

#endif /* UE_CONFIG_H */
