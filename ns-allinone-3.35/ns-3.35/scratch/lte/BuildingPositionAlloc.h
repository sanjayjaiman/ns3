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
#ifndef BUILDING_POSITION_ALLOCATOR_H
#define BUILDING_POSITION_ALLOCATOR_H

#include <iostream>
#include <vector>
#include <memory>
#include "ns3/object.h"
#include "ns3/vector.h"
#include "ns3/node-container.h"
#include "ns3/building.h"
#include "ns3/building-list.h"
#include "ns3/buildings-helper.h"
#include "ns3/building-container.h"
#include <ns3/mobility-building-info.h>
#include <ns3/constant-position-mobility-model.h>
#include "ns3/mobility-module.h"

namespace ns3 {

typedef struct bldg_temp {
  Box box;
  uint32_t num_floors;
} bldg_temp_t;

/**
 * \ingroup mobility
 */
class BuildingPositionAllocator : public Object
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  BuildingPositionAllocator ();
  virtual ~BuildingPositionAllocator () {};

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
   * \param [in] defaultNumFloors The default num floors value to use when reading files
   *             with only X and Y and Z positions.
   * \param [in] delimiter The delimiter character; see CsvReader.
   */
  void Add (const std::string filePath,
            double defaultNumFloors = 0,
            char delimiter = ',');

  uint32_t GetSize (void) const;
  void print (std::ostringstream& os, std::string speperator = "\t");

private:
  void Install(std::vector<bldg_temp_t>& bldg_vector);
  BuildingContainer building_container;
};


} // namespace ns3

#endif /* BUILDING_POSITION_ALLOCATOR_H */
