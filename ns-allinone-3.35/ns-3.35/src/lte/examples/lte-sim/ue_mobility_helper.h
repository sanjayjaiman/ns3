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
 * Author: Sanjay Jaiman <sanjayjaiman@yahoo.com>
 */

#ifndef UE_MOBILITY_HELPER_H
#define UE_MOBILITY_HELPER_H

#include <vector>
#include "ns3/object-factory.h"
#include "ns3/attribute.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/position-allocator.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "UeConfig.h"
#include "EnbConfig.h"
#include <cstdarg>

namespace ns3 {

/**
 * \ingroup mobility
 * \brief Helper class used to assign positions and mobility models to nodes. Extends MobilityHelper.
 *
 * UeMobilityHelper::Install has been overridden to account for "RandonWalk" mobility model.
 */
#define RAND_WALK_MIN_SPEED 60
#define RAND_WALK_MAX_SPEED 60

class UeMobilityHelper : public MobilityHelper
{
public:
  UeMobilityHelper(std::string type);

protected:
  std::string mob_type;
  std::vector<Ptr<UeDesc>> check_mobility_type (NodeContainer c, Ptr<UeConfig> ue_cfg, uint32_t mob_type_cfg);

public:
  static Ptr<UeMobilityHelper> UeMobilityFactory(NodeContainer& node_container, Ptr<ListPositionAllocator> list_position_aloc, Ptr<EnbConfig> enb_cfg, Ptr<UeConfig> ue_cfg, uint32_t type, ...);
};

class ConstPosMobilityHelper : public UeMobilityHelper {
public:
  ConstPosMobilityHelper () : UeMobilityHelper("ns3::ConstantPositionMobilityModel") {
  }
  void install_constant_position_ues (NodeContainer c, uint32_t mask, Ptr<UeConfig> ue_cfg);
};

class RandomWalkMobilityHelper : public UeMobilityHelper {
public:
  RandomWalkMobilityHelper () : UeMobilityHelper("ns3::RandomWalk2dMobilityModel") {
  }
  void install_random_walkers (NodeContainer c, uint32_t mask, Ptr<UeConfig> ue_cfg, std::string min_max_speed);
};

class ConstVelocityMobilityHelper : public UeMobilityHelper {
public:
  ConstVelocityMobilityHelper () : UeMobilityHelper("ns3::ConstantVelocityMobilityModel") {
  }
  void install_constant_velocity_ues (NodeContainer c, uint32_t mask, Ptr<UeConfig> ue_cfg);
};

} // namespace ns3

#endif /* UE_MOBILITY_HELPER_H */
