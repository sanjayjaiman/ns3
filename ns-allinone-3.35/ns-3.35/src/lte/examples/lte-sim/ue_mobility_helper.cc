/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
#include "ns3/rectangle.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/config.h"
#include "ns3/simulator.h"
#include "ns3/names.h"
#include "ns3/string.h"
#include "ue_mobility_helper.h"
#include <iostream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("UeMobilityHelper");
Ptr<UeMobilityHelper> UeMobilityHelper::UeMobilityFactory(NodeContainer& node_container, Ptr<ListPositionAllocator> list_position_aloc,
                                                          Ptr<EnbConfig> enb_cfg, Ptr<UeConfig> ue_cfg, uint32_t type, ...) {
  va_list ap;
  va_start (ap, type);
  Ptr<UeMobilityHelper> p_;
  switch (type) {
  case CONST_POS_ACTIVE_MASK:
  case CONST_POS_IDLE_MASK:
    {
      p_ =  CreateObject<ConstPosMobilityHelper>();
      p_->SetPositionAllocator(list_position_aloc);
      Ptr<ConstPosMobilityHelper> cp_ = DynamicCast<ConstPosMobilityHelper> (p_);
      cp_->install_constant_position_ues(node_container, type, ue_cfg);
      break;
    }
  case RAND_WALK_ACTIVE_MASK:
  case RAND_WALK_IDLE_MASK: 
    {
      std::string min_max = va_arg (ap, char*);
      p_ = CreateObject<RandomWalkMobilityHelper>();
      p_->SetPositionAllocator(list_position_aloc);
      Ptr<RandomWalkMobilityHelper> rw_ = DynamicCast<RandomWalkMobilityHelper> (p_);
      rw_->install_random_walkers(node_container, type, ue_cfg, min_max);
      break;
    }
  case CONST_VELOCITY_ACTIVE_MASK:
  case CONST_VELOCITY_IDLE_MASK:
  {
      p_ = CreateObject<ConstVelocityMobilityHelper>();
      p_->SetPositionAllocator(list_position_aloc);
      Ptr<ConstVelocityMobilityHelper> cv_ = DynamicCast<ConstVelocityMobilityHelper> (p_);
      cv_->install_constant_velocity_ues(node_container, type, ue_cfg);
      break;
  }
  default:
      NS_ASSERT_MSG(true, "Unsupported type");
    break;
  }
  return p_;
}

UeMobilityHelper::UeMobilityHelper(std::string type) : MobilityHelper(), mob_type(type)  {
//  std::string speed = "ns3::UniformRandomVariable[Min=60.0|Max=60.0]";
//  std::string speed = "ns3::UniformRandomVariable[Min=2.0|Max=10.0]";
	SetMobilityModel (type);
  NS_LOG_DEBUG ("Mobility model = " << type);
};

void
RandomWalkMobilityHelper::install_random_walkers (NodeContainer c, uint32_t mask, Ptr<UeConfig> ue_cfg, std::string min_max_speed) {
  std::stringstream speed_ss;
  NS_LOG_FUNCTION (this << min_max_speed);

  speed_ss << "ns3::UniformRandomVariable" << min_max_speed;
  std::string speed = speed_ss.str();
  std::vector<Ptr<UeDesc>> ue_desc_vector = check_mobility_type(c, ue_cfg, mask);
  uint32_t i = 0;
  for (NodeContainer::Iterator iter = c.Begin (); iter != c.End (); ++iter)  {
    Ptr<Node> object = (*iter);
    Ptr<UeDesc> ue_desc_ = ue_desc_vector[i++];
//    put_enb_id(object, ue_desc_->enb_id);
    Ptr<MobilityModel> model = object->GetObject<MobilityModel> ();
    if (model == 0) {
      model = m_mobility.Create ()->GetObject<MobilityModel> ();
      if (model == 0) {
        NS_FATAL_ERROR ("The requested mobility model is not a mobility model: \"" <<
                          m_mobility.GetTypeId ().GetName ()<<"\"");
      }
      object->AggregateObject (model);
    }

    std::string mob_type_ = m_mobility.GetTypeId ().GetName ();
    Vector position = m_position->GetNext ();
    NS_ASSERT_MSG(mob_type == mob_type_, "Mobility model incorrect");

//    std::string mob_name = GetMobilityModelType();
    std::vector<double> walk_rect = ue_desc_->get_rand_walk_rect();
    NS_ASSERT_MSG((walk_rect.size() == 4),
      "UE(" << ue_desc_->pos.x << "," << ue_desc_->pos.y << "," << ue_desc_->pos.z << "): "
      << " rectangle not defined: size = " << walk_rect.size());
    double x_min = walk_rect[0];
    double x_max = walk_rect[1];
    double y_min = walk_rect[2];
    double y_max = walk_rect[3];
    NS_LOG_DEBUG ("install_random_walkers; ID=" << object->GetId() << ", " << ue_desc_->toString() << "; speed range = " << min_max_speed);
    model->SetAttribute("Mode", StringValue ("Time"));
    model->SetAttribute("Time", StringValue ("1s"));
    model->SetAttribute("Speed", StringValue (speed));
    model->SetAttribute("Bounds", RectangleValue (Rectangle (x_min, x_max, y_min, y_max)));
    model->SetPosition (position);    
  }
};

void print_ue_container(std::string header, NodeContainer c) {
  std::stringstream os;
  os << "******* " << header << std::endl;
  for (NodeContainer::Iterator iter = c.Begin (); iter != c.End (); ++iter)  {    
    Ptr<Node> object = (*iter);
    Ptr<UeDesc> ue_ = object->GetObject<UeDescConstantPosActive> ();
    if (ue_ != nullptr) {
      os << "\tUE DESC =  " << ue_->to_string() << std::endl;
    }
  }
  if (c.GetN()) {
    std::cout << os.str();
  }
  else {
    std::cout << "**** " << header << ":: NO UEs" << std::endl;
  }
}
std::vector<Ptr<UeDesc>>
UeMobilityHelper::check_mobility_type (NodeContainer c, Ptr<UeConfig> ue_cfg, uint32_t type_mask) {
  NS_LOG_FUNCTION (this << "type mask= " << type_mask);
  std::vector<Ptr<UeDesc>>& ue_vec_active = ue_cfg->get_active_ue_desc_vector();
  std::vector<Ptr<UeDesc>>& ue_vec_idle = ue_cfg->get_idle_ue_desc_vector();
  std::vector<Ptr<UeDesc>> ue_desc_vector;
  NS_ASSERT_MSG(m_mobility.GetTypeId ().GetName () == mob_type, "Mobility model incorrect: " << mob_type);
  for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vec_active.begin(); iter != ue_vec_active.end(); iter++) {
    Ptr<UeDesc> ue_desc = *iter;
    if (ue_desc->model_type & type_mask) {
      NS_LOG_DEBUG (mob_type << "; Active : " << ue_desc->toString());
      ue_desc_vector.push_back(ue_desc);
    }
  }

  for (std::vector<Ptr<UeDesc>>::iterator iter = ue_vec_idle.begin(); iter != ue_vec_idle.end(); iter++) {
    Ptr<UeDesc> ue_desc = *iter;
    if (ue_desc->model_type & type_mask) {
      NS_LOG_DEBUG (mob_type << "; Idle : " << ue_desc->toString());
      ue_desc_vector.push_back(ue_desc);
   }
  }

  NS_ASSERT_MSG(c.GetN() == ue_desc_vector.size(), "Nodecontainer contains different numnber of nodes from the config: "
        << c.GetN() << " vs " << ue_desc_vector.size());
  return ue_desc_vector;
}

void
ConstPosMobilityHelper::install_constant_position_ues (NodeContainer c, uint32_t mask, Ptr<UeConfig> ue_cfg) {
  NS_LOG_FUNCTION (this);
  std::vector<Ptr<UeDesc>> ue_desc_vector = check_mobility_type(c, ue_cfg, mask);
  uint32_t i = 0;
  std::stringstream str_;
  str_ << "install_constant_position_ues::\n\tNum devices = " << c.GetN() << "\n";
  NS_LOG_DEBUG (str_.str());
  for (NodeContainer::Iterator iter = c.Begin (); iter != c.End (); ++iter)  {    
    Ptr<Node> object = (*iter);
    Ptr<UeDesc> ue_desc_ = ue_desc_vector[i++];
//    put_enb_id(object, ue_desc_->enb_id);
    str_ << "\tID=" << object->GetId() << ", " << ue_desc_->toString() << "\n";
    NS_LOG_DEBUG (str_.str());
  }
  Install(c);
};

void ConstVelocityMobilityHelper::install_constant_velocity_ues (NodeContainer c, uint32_t mask, Ptr<UeConfig> ue_cfg) {
  NS_LOG_FUNCTION (this);
  std::vector<Ptr<UeDesc>> ue_desc_vector = check_mobility_type(c, ue_cfg, mask);
  uint32_t i = 0;
  for (NodeContainer::Iterator iter = c.Begin (); iter != c.End (); ++iter)  {
    Ptr<Node> object = (*iter);
    Ptr<UeDesc> ue_desc_ = ue_desc_vector[i++];
//    put_enb_id(object, ue_desc_->enb_id);
    Ptr<MobilityModel> model_mm = object->GetObject<MobilityModel> ();
    if (model_mm == 0) {
      model_mm = m_mobility.Create ()->GetObject<MobilityModel> ();
      if (model_mm == 0) {
        NS_FATAL_ERROR ("The requested mobility model is not a mobility model: \"" <<
                          m_mobility.GetTypeId ().GetName ()<<"\"");
      }
      object->AggregateObject (model_mm);
    }
    Ptr<ConstantVelocityMobilityModel> model = ns3::DynamicCast<ConstantVelocityMobilityModel>(model_mm);

    std::string mob_type_ = m_mobility.GetTypeId ().GetName ();
    Vector position = m_position->GetNext ();
    NS_ASSERT_MSG(mob_type == mob_type_, "Mobility model incorrect");
    model->SetPosition (position);
    Vector speed = ue_desc_->get_constant_vel_speed();
    model->SetVelocity(speed);
    NS_LOG_DEBUG ("install_constant_velocity_ues; ID=" << object->GetId() << ", " << ue_desc_->toString());
  }
  Install(c); 
}

} // namespace ns3
