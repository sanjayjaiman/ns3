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

#ifndef _RUNTEST_H_
#define _RUNTEST_H_

#include "SimuExe.h"

#define TEST_OUTPUT_FILENAME "test_ue_tput.txt"
#define UE_HEIGHT 1.5

namespace ns3 {

class RunTest : public Object {
  class x_y_struct {
	public:
	  x_y_struct(double y_, std::vector<double> x_vect_) : y(y_), x_vect(x_vect_) {};
	  double y;
	  std::vector<double> x_vect;
  };
public:
	RunTest(Ptr<SimulationParameters>& sim_params_, std::string output_file_ = TEST_OUTPUT_FILENAME);
	void run();
private:
	void build_range_vec();
private:
	std::string x_hdr_line;
	std::string output_file;
	Ptr<SimulationParameters>& sim_params;
	uint32_t num_ues_in_test;
	std::vector<x_y_struct> x_y;
};

};

#endif