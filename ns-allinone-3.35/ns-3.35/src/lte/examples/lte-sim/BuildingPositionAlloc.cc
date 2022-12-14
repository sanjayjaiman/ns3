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


#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/csv-reader.h"
#include "BuildingPositionAlloc.h"

#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("BuildingPosAllocator");

NS_OBJECT_ENSURE_REGISTERED (BuildingPositionAllocator);

TypeId
BuildingPositionAllocator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::BuildingPositionAllocator")
    .SetParent<Object> ()
    .SetGroupName ("Mobility")
    .AddConstructor<BuildingPositionAllocator> ()
    ;
  return tid;
}

BuildingPositionAllocator::BuildingPositionAllocator () {
};

void
BuildingPositionAllocator::Install(std::vector<bldg_temp_t>& bldg_vector) {
  uint32_t i = 0;
  for (std::vector<bldg_temp_t>::iterator iter = bldg_vector.begin(); iter != bldg_vector.end(); iter++) {
    bldg_temp_t bldg_ = *iter;
    Box& b = bldg_.box;
    Ptr<Building> bld_ = CreateObject<Building>();  // Automatically adds to global static "BuildingList"
    bld_->SetBoundaries(b);
    bld_->SetBuildingType (Building::Residential);
    bld_->SetExtWallsType (Building::ConcreteWithWindows);
    bld_->SetNFloors(bldg_.num_floors);
    // building_container is simply storing buildings.
    // Static "BuildingList" is what is being used to configure buildings in the simulation.
    building_container.Add(bld_);
//    mobility.AggregateObject(bld_);
//		Ptr<MobilityModel> a = CreateObject<ConstantPositionMobilityModel> ();
//  	nodes.Get (i)->AggregateObject (a);
    NS_LOG_INFO ("Adding Building" << i << ":(" << b.xMin << ", " << b.xMax << ", " <<b.yMin << ", " <<b.yMax << ", " <<b.zMin << ", " <<b.zMax << ")");
    i++;
  }
}


void
BuildingPositionAllocator::Add (const std::string filePath,
                            double defaultNumFloors /* = 0 */,
                            char delimiter /* = ',' */)
{
  NS_LOG_FUNCTION (this << filePath << std::string ("'") + delimiter + "'");

  std::vector<bldg_temp_t> bldg_vector_;

  CsvReader csv (filePath, delimiter);
  uint32_t i = 0;

  
  while (csv.FetchNextRow ())  {
    std::ostringstream os;

    uint32_t col_count = csv.ColumnCount ();
    if (col_count == 1 || col_count == 0) {
//        NS_LOG_INFO ("** Comment line **"); 
        // comment line
        continue;
    }
  
    double x_min, y_min, z_min;
    double x_max, y_max, z_max;
    uint32_t num_floors;
    bool ok = csv.GetValue (0, x_min);
    os << "x_min: " << x_min << (ok ? " ok" : " FAIL") << ", ";
    NS_ASSERT_MSG (ok, "failed reading x_min");
    ok = csv.GetValue (1, x_max);
    os << "x_max: " << x_max << (ok ? " ok" : " FAIL") << ", ";
    NS_ASSERT_MSG (ok, "failed reading x_max");
    ok = csv.GetValue (2, y_min);
    os << "y_min = " << y_min << (ok ? " ok" : " FAIL") << ", ";
    NS_ASSERT_MSG (ok, "failed reading y_min");
    ok = csv.GetValue (3, y_max);
    os << "y_max = " << y_max << (ok ? " ok" : " FAIL") << ", ";
    NS_ASSERT_MSG (ok, "failed reading y_max");
    ok = csv.GetValue (4, z_min);
    os << "z_min = " << z_min << (ok ? " ok" : " FAIL") << ", ";
    NS_ASSERT_MSG (ok, "failed reading z_min");
    ok = csv.GetValue (5, z_max);
    os << "z_max = " << z_max << (ok ? " ok" : " FAIL") << ", ";
    NS_ASSERT_MSG (ok, "failed reading z_max");
    if (csv.ColumnCount () > 6) {
        ok = csv.GetValue (6, num_floors);
        os << "num floors = " << num_floors << (ok ? " ok" : " FAIL");
        NS_ASSERT_MSG (ok, "failed reading num floors");
    }
    else {
        num_floors = defaultNumFloors;
        os << "num floors = " << num_floors << "(Default)";
    }
    Box box_ = Box(x_min, x_max, y_min, y_max, z_min, z_max);
    bldg_vector_.push_back({box_, num_floors});

    NS_LOG_INFO ("Row#" << i << ": " << os.str());
    i++;
  }  // while FetchNextRow
  NS_LOG_INFO ("Read: " << csv.RowNumber () << " rows");

  Install(bldg_vector_);
}


uint32_t
BuildingPositionAllocator::GetSize (void) const
{
  return BuildingList::GetNBuildings ();
}


void
BuildingPositionAllocator::print (std::ostringstream& os, std::string seperator)
{
//		uint32_t num_buildings_allocated = GetSize();
		uint32_t i = 0;
		os  <<"Num"<<seperator<<"Xmin"<<seperator<<"Ymin"<<seperator<<"zmin"<<seperator<<"Xmax"<<seperator<<"Ymax"<<seperator<<"XMax"<<seperator<<"Num Floors" << std::endl;
		for (BuildingList::Iterator iter = BuildingList::Begin(); iter !=  BuildingList::End(); iter++) {
			Ptr<Building> building = *iter;
      Box box_ = building->GetBoundaries ();
			os << i  << seperator <<box_.xMin << seperator <<box_.yMin  << seperator << box_.zMin  << seperator << 
          box_.xMax  << seperator << box_.yMax  << seperator << box_.zMax << seperator << building->GetNFloors() << std::endl;
			i++;
		}
}

}