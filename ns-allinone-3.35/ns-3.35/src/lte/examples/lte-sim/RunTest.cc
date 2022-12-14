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

#include "RunTest.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RunTest");

NS_OBJECT_ENSURE_REGISTERED (RunTest);


void get_int_range_values(std::string param, std::vector<int32_t>& range_x) {
	StringValue strValue;
	GlobalValue::GetValueByName (param, strValue);
	std::string range_str = strValue.Get ();
	std::string delimiter = "|";
	uint32_t pos_start = 0U;
	std::size_t pos_end = range_str.find (delimiter, pos_start);
	if (pos_end == std::string::npos) {
		std::cout << "Invalid range specified" << std::endl;
		exit(0);
	}
	std::string range1 = range_str.substr (pos_start, pos_end - pos_start);
	pos_start = pos_end + delimiter.length();
	pos_end = range_str.find (delimiter, pos_start);
	std::string range2 = range_str.substr (pos_start, pos_end - pos_start);
	pos_start = pos_end + delimiter.length();
	pos_end = range_str.length();
	std::string step_str= range_str.substr (pos_start, pos_end - pos_start);
//		std::cout << "range = " << range1 << "|" << range2 << "; spre = " << step_str << std::endl;
	std::stringstream s1(range1);
	std::stringstream s2(range2);
	std::stringstream st(step_str);
	int32_t r1, r2, step;
	s1 >> r1; s2 >> r2; st >> step;
	range_x.push_back(r1);
	range_x.push_back(r2);
	range_x.push_back(step);
}

#define MIN_DISTANCE_FROM_ENB 10
#define ADJUST_MIN_DISTANCE false

void  RunTest::build_range_vec() {
	std::vector<int32_t> range_x;
	std::vector<int32_t> range_y;

	get_int_range_values("SimulationTest_X_Range", range_x);
	get_int_range_values("SimulationTest_Y_Range", range_y);

	int32_t r1_x = range_x[0];
	int32_t r2_x = range_x[1];
	int32_t step_x = range_x[2];
	
	int32_t r1_y = range_y[0];
	int32_t r2_y = range_y[1];
	int32_t step_y = range_y[2];
//		std::cout << "range = " << r1_x << "|" << r2_x << "; spre = " << step_x << std::endl;	
	for (double y = r2_y; y >= r1_y; y = y-step_y) {
		std::vector<double> x_vec;
		double y_ = y;
		for (double x = r1_x; x <= r2_x; x += step_x) {
			if (ADJUST_MIN_DISTANCE) {
				while (sqrt(pow(x,2) + pow(y_,2)) < MIN_DISTANCE_FROM_ENB) {
					y_ += MIN_DISTANCE_FROM_ENB;
				}
			}
			x_vec.push_back(x);
		}
		if (x_hdr_line == "") {
			std::stringstream top_line;
			for (std::vector<double>::iterator i = x_vec.begin(); i != x_vec.end(); i++) {
				double x = *i;
				top_line << "," << x;
			}
			top_line << std::endl;
			x_hdr_line = top_line.str();
		}
		x_y.push_back(x_y_struct(y_, x_vec));
	}
}

RunTest::RunTest(Ptr<SimulationParameters>& sim_params_, std::string output_file_) : 
    x_hdr_line(""), output_file(output_file_), sim_params(sim_params_) {
    IntegerValue intValue;
    GlobalValue::GetValueByName ("SimulationTestModeNumUEs", intValue);
    num_ues_in_test = intValue.Get ();
    std::cout << "num_ues_in_test = " << num_ues_in_test << std::endl;
    build_range_vec();
};

void RunTest::run() {
	bool first_time = true;
	bool print_it = sim_params->verbose >= 1;
	for (std::vector<x_y_struct>::iterator iter = x_y.begin(); iter != x_y.end(); iter++) {
		double y = iter->y;
		std::vector<double> x_vec = iter->x_vect;
		std::ostringstream line_str_;
		line_str_.precision(4);
		line_str_.str(std::string());
		std::stringstream top_line;
		for (std::vector<double>::iterator i = x_vec.begin(); i != x_vec.end(); i++) {
			double x = *i;
			Ptr<UeConfig> ue_cfg = CreateObject<UeConfig>();
			// Add UEs at the same spot for testing
			for (uint32_t i = 0; i < num_ues_in_test; i++) {
				ue_cfg->add_active_ue(x, y, UE_HEIGHT);
			}
			ue_cfg->init();
			//Connect to single ENB because we are moving the UE all the time
			SimuExe sim_exe(sim_params, ue_cfg);

			std::ostringstream bld_str;
			sim_exe.print_buildings_info_comma_seperated(bld_str);				
			sim_exe.run();
			if (first_time) {
				first_time = false;
				std::ostringstream os;
				sim_exe.print_enb_info(os);
				os << std::endl;
				os << "Num UEs = " << num_ues_in_test << std::endl;
				sim_exe.print_ue_info(os);
				os << std::endl;
				sim_exe.print_path_loss_model(os, true);
				os << std::endl;
				std::ofstream myfile;
				myfile.open (output_file);
				myfile << os.str();
				myfile << bld_str.str();
				myfile << std::endl << std::endl;
				myfile << x_hdr_line;
				myfile.close();
				std::cout << "Running Test simulation (buildings" << 
				((sim_params->use_buildings()) ? " present" : " not present") << ") with UEs at:" << std::endl;
			}
			sim_params->verbose = 0;  // We just want to print debug info once
			std::cout << "(" << x << "," << y << "), " << std::flush;
			NS_ASSERT(sim_exe.get_num_ues() == num_ues_in_test);  // There should only be one UE in the map
//				sim_exe.print_ue_pos(os);
			sim_exe.print_ue_tput(line_str_);
		}
		std::cout << std::endl;
		if (print_it) {
			std::cout << line_str_.str() << std::endl;
		}
		line_str_ << std::endl;
		std::ofstream myfile;
		myfile.open (output_file, std::ios::app);
		myfile << y << "," << line_str_.str();
		myfile.close();
	}
	std::cout << "Written file " << output_file << std::endl;
}

}
