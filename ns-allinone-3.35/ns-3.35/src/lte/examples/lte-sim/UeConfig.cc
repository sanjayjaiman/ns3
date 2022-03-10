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
 * Author: Sanjay Jaiman <sjaiman@parallelwireless.com>
 */

#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/csv-reader.h"
#include "UeConfig.h"

#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("UeConfig");

NS_OBJECT_ENSURE_REGISTERED (UeConfig);

TypeId
UeConfig::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UeConfig")
    .SetParent<Object> ()
    .SetGroupName ("Mobility")
    .AddConstructor<UeConfig> ()
    ;
  return tid;
}


UeDescConstantPosActive::UeDescConstantPosActive(uint32_t id_, Vector pos_) : UeDescConstantPos(id_, pos_, true, CONST_POS_ACTIVE_MASK) {
};

UeDescConstantPosIdle::UeDescConstantPosIdle(uint32_t id_, Vector pos_) : UeDescConstantPos(id_, pos_, false, CONST_POS_IDLE_MASK) {
};

UeDescRandWalk::UeDescRandWalk(uint32_t id_, Vector pos_, bool active, std::vector<double> walk_rect_, uint32_t mask) : 
      UeDesc(id_, pos_, active, mask), walk_rect(walk_rect_) {
};

UeDescRandWalkActive::UeDescRandWalkActive(uint32_t id_, Vector pos_, std::vector<double> walk_rect_) : 
      UeDescRandWalk(id_, pos_, true, walk_rect_, RAND_WALK_ACTIVE_MASK) {
};

UeDescRandWalkIdle::UeDescRandWalkIdle(uint32_t id_, Vector pos_, std::vector<double> walk_rect_) : 
      UeDescRandWalk(id_, pos_, false, walk_rect_, RAND_WALK_IDLE_MASK) {
};

UeDescConstantVel::UeDescConstantVel(uint32_t id_, Vector pos_, bool active, Vector speed_, uint32_t mask) : 
      UeDesc(id_, pos_, active, mask), speed(speed_) {
};

UeDescConstantVelActive::UeDescConstantVelActive(uint32_t id_, Vector pos_, Vector speed) :
      UeDescConstantVel(id_, pos_, true, speed, CONST_VELOCITY_ACTIVE_MASK) {
};

UeDescConstantVelIdle::UeDescConstantVelIdle(uint32_t id_, Vector pos_, Vector speed) :
      UeDescConstantVel(id_, pos_, false, speed, CONST_VELOCITY_IDLE_MASK) {
};

UeConfig::UeConfig () {
};
UeConfig::UeConfig (std::string ue_pos_file) {
  add_ues(ue_pos_file);
}

void UeConfig::init() {
  cp_active_list_position = CreateObject<ListPositionAllocator>();
  cp_idle_list_position =  CreateObject<ListPositionAllocator>();
  rw_active_list_position =  CreateObject<ListPositionAllocator>();
  rw_idle_list_position =  CreateObject<ListPositionAllocator>();
  cv_active_list_position =  CreateObject<ListPositionAllocator>();
  cv_idle_list_position =  CreateObject<ListPositionAllocator>();
  make_list_position(cp_active_list_position, CONST_POS_ACTIVE_MASK);
  make_list_position(cp_idle_list_position, CONST_POS_IDLE_MASK);
  make_list_position(rw_active_list_position, RAND_WALK_ACTIVE_MASK);
  make_list_position(rw_idle_list_position, RAND_WALK_IDLE_MASK);
  make_list_position(cv_active_list_position, CONST_VELOCITY_ACTIVE_MASK);
  make_list_position(cv_idle_list_position, CONST_VELOCITY_IDLE_MASK);
}

Ptr<ListPositionAllocator> UeConfig::make_list_position(Ptr<ListPositionAllocator>& list_, uint32_t mask) {
  for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vector_active.begin(); iter != ue_vector_active.end(); iter++) {
    Ptr<UeDesc> ue_ = *iter;
    if (ue_->model_type == mask) {
      list_->Add(ue_->pos);
    }
  }
  for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vector_idle.begin(); iter != ue_vector_idle.end(); iter++) {
    Ptr<UeDesc> ue_ = *iter;
    if (ue_->model_type == mask) {
      list_->Add(ue_->pos);
    }
  }
  return list_;
}

void
UeConfig::add_active_ue (double x, double y, double z)
{
    Ptr<UeDesc> ue_;
    Vector pos_ = Vector(x, y, z);
    ue_ = CreateObject<UeDescConstantPosActive>(0, pos_);
    ue_vector_active.push_back(ue_);
}

void
UeConfig::add_ues (const std::string filePath, char delimiter /* = ',' */)
{
  NS_LOG_FUNCTION (this << filePath << std::string ("'") + delimiter + "'");
  #define MIN_COL_COUNT 3
  #define MAX_COL_COUNT 9

  CsvReader csv (filePath, delimiter);
  uint32_t row = 0;
  uint32_t line_num = 0;
  uint32_t id_ = 0;
  uint32_t enb_id_ = 0;
  while (csv.FetchNextRow ())  {
    line_num++;
    std::ostringstream os;
    uint32_t col_count = csv.ColumnCount ();
    if (col_count == 1 || col_count == 0) {
//        NS_LOG_INFO ("** Comment line **"); 
        // comment line
        continue;
    }
    Vector pos_;
    uint32_t i = 0;
    bool ok;
    NS_ASSERT_MSG ((col_count >= MIN_COL_COUNT+1 && col_count <= MAX_COL_COUNT), 
                    "File:" << filePath <<"::Invalid number of tokens in line " << line_num);
    ok = csv.GetValue (i++, id_);
    os << "id: " << id_ << ", ";
    ok = csv.GetValue (i++, pos_.x);
    os << "x: " << pos_.x << ", ";
    NS_ASSERT_MSG (ok, "failed reading x");
    ok = csv.GetValue (i++, pos_.y);
    os << "y = " << pos_.y << ", ";
    NS_ASSERT_MSG (ok, "failed reading y");
    ok = csv.GetValue (i++, pos_.z);
    NS_ASSERT_MSG (ok, "failed reading z");
    os << "z = " << pos_.z << "; ";
//    ok = csv.GetValue (i++, enb_id_);
//    NS_ASSERT_MSG (ok, "failed reading ENB ID");
//    os << "ENB ID = " << enb_id_ << "; ";
    Ptr<UeDesc> ue_;
    bool is_active = true;
    if (col_count == MIN_COL_COUNT || col_count == MIN_COL_COUNT+1) {
      ue_ = CreateObject<UeDescConstantPosActive>(id_, pos_);
    }
    else {
      std::string model_type_;      
      ok = csv.GetValue (i++, model_type_);
      os << "UE Type = " << model_type_ << ", ";
      if (model_type_ ==  CONST_POS_IDLE_STR) {
        ue_ = CreateObject<UeDescConstantPosIdle>(id_, pos_);
        is_active = false;
      }
      else if (model_type_ ==  RAND_WALK_STR || model_type_ ==  RAND_WALK_IDLE_STR) {
        double x_min, y_min;
        double x_max, y_max;
        bool ok = csv.GetValue (i++, x_min);
        os << "x_min: " << x_min << ", ";
        NS_ASSERT_MSG (ok, "failed reading x_min");
        ok = csv.GetValue (i++, x_max);
        os << "x_max: " << x_max << ", ";
        NS_ASSERT_MSG (ok, "failed reading x_max");
        ok = csv.GetValue (i++, y_min);
        os << "y_min = " << y_min << ", ";
        NS_ASSERT_MSG (ok, "failed reading y_min");
        ok = csv.GetValue (i++, y_max);
        os << "y_max = " << y_max << ", ";
        NS_ASSERT_MSG (ok, "failed reading y_max");

        std::vector<double> walk_rect;
        walk_rect.push_back(x_min);
        walk_rect.push_back(x_max);
        walk_rect.push_back(y_min);
        walk_rect.push_back(y_max);
        if (model_type_ ==  RAND_WALK_STR) {
          ue_ = CreateObject<UeDescRandWalkActive>(id_, pos_, walk_rect);
        }
        else {
          ue_ = CreateObject<UeDescRandWalkIdle>(id_, pos_, walk_rect);
          is_active = false;
        }
      }
      else if (model_type_ ==  CONST_VELOCITY_STR || model_type_ ==  CONST_VELOCITY_IDLE_STR) {
        Vector speed;
        bool ok = csv.GetValue (i++,speed.x);
        os << "speed.x: " <<speed.x<< ", ";
        NS_ASSERT_MSG (ok, "failed reading x_min");
        ok = csv.GetValue (i++,speed.y);
        os << "speed.y: " <<speed.y << ", ";
        NS_ASSERT_MSG (ok, "failed readingspeed.y");
        ok = csv.GetValue (i++,speed.z);
        os << "speed.z = " <<speed.z << ", ";
        NS_ASSERT_MSG (ok, "failed readingspeed.z");        
        if (model_type_ ==  CONST_VELOCITY_STR) {
          ue_ = CreateObject<UeDescConstantVelActive>(id_, pos_, speed);
        }
        else {
          ue_ = CreateObject<UeDescConstantVelIdle>(id_, pos_, speed);
          is_active = false;
        }
      }
    }
    ue_->set_enb_id(enb_id_);
    if (is_active) {
      ue_vector_active.push_back(ue_);
    }
    else {
      ue_vector_idle.push_back(ue_);
    }
    NS_LOG_INFO ("Read: Row#" << row << ": " << os.str());
    row++;
    
  }  // while FetchNextRow
  NS_LOG_INFO ("Read: " << csv.RowNumber () << " rows");
	uint32_t i = 0;
  for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vector_active.begin(); iter != ue_vector_active.end(); iter++) {
    Ptr<UeDesc> ue_ = *iter;
    NS_LOG_INFO ("Adding Active Ue" << i << "; " << ue_->toString());
    i++;
  }
  for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vector_idle.begin(); iter != ue_vector_idle.end(); iter++) {
    Ptr<UeDesc> ue_ = *iter;
    NS_LOG_INFO ("Adding Idle Ue" << i << "; " << ue_->toString());
    i++;
  }
}


uint32_t
UeConfig::GetSize (void) const
{
  return ue_vector_active.size();
}


void
UeConfig::print (std::ostringstream& os)
{
		uint32_t num_enbs_allocated = GetSize();
    os << "\n================================================================================" << std::endl;
    os << "UE Config:: Num Ues = " << num_enbs_allocated << std::endl;
		for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vector_active.begin(); iter !=  ue_vector_active.end(); iter++) {
      Ptr<UeDesc> ue_ = *iter;
      os << ue_->toString() << std::endl;
		}
		for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vector_idle.begin(); iter !=  ue_vector_idle.end(); iter++) {
      Ptr<UeDesc> ue_ = *iter;
      os << ue_->toString() << std::endl;
		}
    os << "================================================================================" << std::endl;
}

bool UeConfig::get_ue_struct(Ptr<UeDesc>& ret, uint32_t index) {
		uint32_t i = 0;
		for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vector_active.begin(); iter !=  ue_vector_active.end(); iter++) {
      Ptr<UeDesc> ue_ = *iter;
      if (index == i) {
			  ret = ue_;
        return true;
      }
      i++;
    }
    return false;
}

}