#include <stdexcept>
#include "../sim/Engine.h"
#include "NVM_PHY_ONFI_NVDDR2.h"
#include "Stats.h"

namespace SSD_Components {
	/*hack: using this style to emulate event/delegate*/
	NVM_PHY_ONFI_NVDDR2* NVM_PHY_ONFI_NVDDR2::_my_instance;

	NVM_PHY_ONFI_NVDDR2::NVM_PHY_ONFI_NVDDR2(const sim_object_id_type& id, ONFI_Channel_NVDDR2** channels,
		unsigned int ChannelCount, unsigned int chip_no_per_channel, unsigned int DieNoPerChip, unsigned int PlaneNoPerDie)
		: NVM_PHY_ONFI(id, ChannelCount, chip_no_per_channel, DieNoPerChip, PlaneNoPerDie), channels(channels), channel_num(ChannelCount)//channels是由SSD_Device用已经初始化好的Channels传入的
	{
		WaitingReadTX = new Flash_Transaction_Queue[channel_count];
		WaitingGCRead_TX = new Flash_Transaction_Queue[channel_count];
		WaitingMappingRead_TX = new Flash_Transaction_Queue[channel_count];
		WaitingCopybackWrites = new std::list<DieBookKeepingEntry*>[channel_count];
		bookKeepingTable = new ChipBookKeepingEntry * [channel_count];
		for (unsigned int channelID = 0; channelID < channel_count; channelID++) {
			bookKeepingTable[channelID] = new ChipBookKeepingEntry[chip_no_per_channel];
			for (unsigned int chipID = 0; chipID < chip_no_per_channel; chipID++) {
				bookKeepingTable[channelID][chipID].Expected_command_exec_finish_time = T0;
				bookKeepingTable[channelID][chipID].Last_transfer_finish_time = T0;
				bookKeepingTable[channelID][chipID].Die_book_keeping_records = new DieBookKeepingEntry[DieNoPerChip];
				bookKeepingTable[channelID][chipID].Status = ChipStatus::IDLE;
				bookKeepingTable[channelID][chipID].HasSuspend = false;
				bookKeepingTable[channelID][chipID].WaitingReadTXCount = 0;
				bookKeepingTable[channelID][chipID].No_of_active_dies = 0;
				for (unsigned int dieID = 0; dieID < DieNoPerChip; dieID++) {
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].ActiveCommand = NULL;
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].ActiveTransactions.clear();
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].SuspendedCommand = NULL;
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].SuspendedTransactions.clear();
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].Free = true;
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].Suspended = false;
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].DieInterleavedTime = INVALID_TIME;
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].Expected_finish_time = INVALID_TIME;
					bookKeepingTable[channelID][chipID].Die_book_keeping_records[dieID].RemainingExecTime = INVALID_TIME;
				}
			}
		}

		//下面对记录2D-Mesh拓扑结构状态的Link_state进行初始化

		Link_state = new int** [ChannelCount];                           //第一维度是行数
		for (int i = 0; i < ChannelCount; i++) {                          //第二维度是列
			Link_state[i] = new int* [chip_no_per_channel];
			for (int j = 0; j < chip_no_per_channel; j++) {              //第三维度代表的是纵向的还是横向的链路
				Link_state[i][j] = new int[2];
			}
		}

		for (int i = 0; i < ChannelCount; i++) {
			for (int j = 0; j < chip_no_per_channel; j++) {
				for (int k = 0; k < 2; k++) {
					if (i == ChannelCount - 1 && k == 1) {              //k为0表示左横向链路，k为1表示下纵向链路
						Link_state[i][j][k] = -1;                       //-1表示该链路无效，因为最下方的芯片没有下纵向链路
					}
					else {
						Link_state[i][j][k] = 0;		                //0表示该链路空闲
					}
				}
			}
		}

		//下面对记录控制器状态的数组进行初始化
		Controller_state = new int[ChannelCount];
		for (int i = 0; i < ChannelCount; i++) {
			Controller_state[i] = 0;
		}

		_my_instance = this;
	}

	void NVM_PHY_ONFI_NVDDR2::Setup_triggers()
	{
		Sim_Object::Setup_triggers();
		for (unsigned int i = 0; i < channel_count; i++) {
			for (unsigned int j = 0; j < chip_no_per_channel; j++) {
				channels[i]->Chips[j]->Connect_to_chip_ready_signal(handle_ready_signal_from_chip);
			}
		}
	}

	void NVM_PHY_ONFI_NVDDR2::Validate_simulation_config()
	{
	}

	void NVM_PHY_ONFI_NVDDR2::Start_simulation()
	{
	}

	inline BusChannelStatus NVM_PHY_ONFI_NVDDR2::Get_channel_status(flash_channel_ID_type channelID)
	{
		return channels[channelID]->GetStatus();
	}

	//通过下面这个函数检查该chip的所有端口是否都尝试过，如果都尝试过则回溯
	bool NVM_PHY_ONFI_NVDDR2::Backtracking_or_not(int*** Route_port_record, int current_channelID, int current_chipID) {
		bool flag = true;
		for (int i = 0; i < 4; i++) {
			if (Route_port_record[current_channelID][current_chipID][i] == 0) {
				flag = false;													//如果还能找到没有尝试过的链路，则不需要回溯
				break;
			}
		}
		return flag;

	}

	//反转端口方向
	int NVM_PHY_ONFI_NVDDR2::port_inversion(int port_num) {
		if (port_num < 2) {
			return port_num + 2;
		}
		else {
			return port_num - 2;
		}
	}
	//根据当前的位置及数据传输的端口，计算出下一个chip的位置,直接改变current_channelID及current_chipID的值
	void NVM_PHY_ONFI_NVDDR2::update_coordinate(int& current_channelID, int& current_chipID, int port_direction) {
		if (port_direction == 0) {
			current_channelID = current_channelID - 1;
		}
		else if (port_direction == 1) {
			current_chipID = current_chipID + 1;
		}
		else if (port_direction == 2) {
			current_channelID = current_channelID + 1;
		}
		else {
			current_chipID = current_chipID - 1;
		}
	}
	std::pair<int, int> NVM_PHY_ONFI_NVDDR2::get_updated_coordinate(int current_channelID, int current_chipID, int port_direction) {      //用于获取chipid的值，确保路径不会预定到别的控制器与控制器相连的第一个链路
		if (port_direction == 0) {
			return std::make_pair(current_channelID - 1, current_chipID);
		}
		else if (port_direction == 1) {
			return std::make_pair(current_channelID, current_chipID + 1);
		}
		else if (port_direction == 2) {
			return std::make_pair(current_channelID + 1, current_chipID);
		}
		else {
			return std::make_pair(current_channelID, current_chipID - 1);
		}
	}
	//该函数用于寻找当前芯片的最佳的端口尝试顺序，y坐标相同默认先向上走，x坐标相同默认先向右边走。当前芯片与目标为斜对角时，优先水平方向，后考虑纵向方向
	void NVM_PHY_ONFI_NVDDR2::get_the_best_port_attempt_order(PortDirection* attempt_sequence, flash_channel_ID_type target_channelID, flash_chip_ID_type target_chip_id, flash_channel_ID_type current_channelID, flash_chip_ID_type current_chip_id) {
		if (current_channelID == target_channelID && current_chip_id < target_chip_id) {			//在目标芯片的正左边
			attempt_sequence[0] = PortDirection::Port_RIGHT;
			attempt_sequence[1] = PortDirection::Port_UP;
			attempt_sequence[2] = PortDirection::Port_DOWN;
			attempt_sequence[3] = PortDirection::Port_LEFT;
		}
		else if (current_channelID == target_channelID && current_chip_id > target_chip_id) {		//在目标芯片的正右边
			attempt_sequence[0] = PortDirection::Port_LEFT;
			attempt_sequence[1] = PortDirection::Port_UP;
			attempt_sequence[2] = PortDirection::Port_DOWN;
			attempt_sequence[3] = PortDirection::Port_RIGHT;
		}
		else if (current_chip_id == target_chip_id && current_channelID > target_channelID) {		//在目标芯片正下方
			attempt_sequence[0] = PortDirection::Port_UP;
			attempt_sequence[1] = PortDirection::Port_RIGHT;
			attempt_sequence[2] = PortDirection::Port_LEFT;
			attempt_sequence[3] = PortDirection::Port_DOWN;
		}
		else if (current_chip_id == target_chip_id && current_channelID < target_channelID) {		//在目标芯片正上方
			attempt_sequence[0] = PortDirection::Port_DOWN;
			attempt_sequence[1] = PortDirection::Port_RIGHT;
			attempt_sequence[2] = PortDirection::Port_LEFT;
			attempt_sequence[3] = PortDirection::Port_UP;
		}
		else if (current_channelID < target_channelID && current_chip_id < target_chip_id) {	    //在目标芯片左上方
			attempt_sequence[0] = PortDirection::Port_RIGHT;
			attempt_sequence[1] = PortDirection::Port_DOWN;
			attempt_sequence[2] = PortDirection::Port_LEFT;
			attempt_sequence[3] = PortDirection::Port_UP;
		}
		else if (current_channelID < target_channelID && current_chip_id > target_chip_id) {		//右上方
			attempt_sequence[0] = PortDirection::Port_LEFT;
			attempt_sequence[1] = PortDirection::Port_DOWN;
			attempt_sequence[2] = PortDirection::Port_RIGHT;
			attempt_sequence[3] = PortDirection::Port_UP;
		}
		else if (current_channelID > target_channelID && current_chip_id > target_chip_id) {        //右下方
			attempt_sequence[0] = PortDirection::Port_LEFT;
			attempt_sequence[1] = PortDirection::Port_UP;
			attempt_sequence[2] = PortDirection::Port_RIGHT;
			attempt_sequence[3] = PortDirection::Port_DOWN;
		}
		else {																						//左下方
			attempt_sequence[0] = PortDirection::Port_RIGHT;
			attempt_sequence[1] = PortDirection::Port_UP;
			attempt_sequence[2] = PortDirection::Port_LEFT;
			attempt_sequence[3] = PortDirection::Port_DOWN;
		}
		return;
	}

	//下面这段代码用于找出芯片及对应端口的链路在Link_state数组的坐标
	void NVM_PHY_ONFI_NVDDR2::Port_index_converted_to_link_index(int port_channelID, int port_chipID, int port_direction, int* check_array) {
		if (port_direction == 0) {
			check_array[0] = port_channelID - 1;
			check_array[1] = port_chipID;
			check_array[2] = 1;
		}
		else if (port_direction == 1) {
			check_array[0] = port_channelID;
			check_array[1] = port_chipID + 1;
			check_array[2] = 0;
		}
		else if (port_direction == 2) {
			check_array[0] = port_channelID;
			check_array[1] = port_chipID;
			check_array[2] = 1;
		}
		else {
			check_array[0] = port_channelID;
			check_array[1] = port_chipID;
			check_array[2] = 0;
		}
	}

	//下面这段代码负责寻找最合适的控制器
	int NVM_PHY_ONFI_NVDDR2::Find_the_most_suitable_controller(flash_channel_ID_type channelID) {
		int i = channelID;									//先使得i初始化为最近的控制器，如果没有的话，就向上或向下寻找离得最近的空闲控制器
		if (Controller_state[i] == 0) {
			return i;
		}
		int j = i - 1;
		int k = i + 1;
		while (j >= 0 || k < channel_num) {
			if (j >= 0) {
				if (Controller_state[j] == 0) {
					return j;
				}
			}
			if (k < channel_num) {
				if (Controller_state[k] == 0) {
					return k;
				}
			}
			j--;
			k++;
		}
		return -1;											//当程序进行到这里说明找不到合适的控制器
	}
	bool NVM_PHY_ONFI_NVDDR2::Determines_whether_the_wait_queue_can_be_executed(const std::vector<int>& path, int*** Link_state, int controller_num) {
		if (path.size()==0) {
			return false;
		}
		if (controller_num<0||controller_num>channel_num) {
			return false;
		}
		bool flag = true;
		int current_channel = controller_num;
		int current_chip = 0;
		int path_size = path.size();
		for (int i = 0; i < path_size; i++) {
			int check_array[3];
			Port_index_converted_to_link_index(current_channel, current_chip, port_inversion(path[i]), check_array);
			if (Link_state[check_array[0]][check_array[1]][check_array[2]] == 1) {	
				flag = false;
				break;
			}
			if (i != path_size - 1) {
				update_coordinate(current_channel, current_chip, path[i + 1]);
			}
		}
		return flag;
	}

	void NVM_PHY_ONFI_NVDDR2::Set_link_status_based_on_path(const std::vector<int>& path, int*** Link_state, int controller_num) {   //这个函数用于根据path数组的内容，改变Link_state中被预定路径的链路状态
		int current_channel = controller_num;
		int current_chip = 0;
		int path_size = path.size();
		for (int i = 0; i < path_size; i++) {
			int check_array[3];
			Port_index_converted_to_link_index(current_channel, current_chip, port_inversion(path[i]), check_array);
			Link_state[check_array[0]][check_array[1]][check_array[2]] = 1;
			if (i != path_size - 1) {
				update_coordinate(current_channel, current_chip, path[i + 1]);
			}
		}
	}

	void NVM_PHY_ONFI_NVDDR2::Set_link_status_to_idle_based_on_the_path(const std::vector<int>& path, int*** Link_state, int controller_num) {   //当请求完成之后，将请求使用的路径设置为空闲
		int current_channel = controller_num;
		int current_chip = 0;
		int path_size = path.size();
		for (int i = 0; i < path_size; i++) {
			int check_array[3];
			Port_index_converted_to_link_index(current_channel, current_chip, port_inversion(path[i]), check_array);
			Link_state[check_array[0]][check_array[1]][check_array[2]] = 0;
			if (i != path_size - 1) {
				update_coordinate(current_channel, current_chip, path[i + 1]);
			}
		}
	}

	bool NVM_PHY_ONFI_NVDDR2::whether_a_ring_is_formed(std::vector<std::pair<int, int>> Passed_coordinates, std::pair<int, int>coordinate) {
		bool flag = false;
		auto it = std::find(Passed_coordinates.begin(), Passed_coordinates.end(), coordinate);
		if (it != Passed_coordinates.end()) {
			flag = true;						//说明形成了环路
		}
		else {
			flag = false;						//未形成环路
		}
		return flag;
	}
	bool NVM_PHY_ONFI_NVDDR2::check_idle_controller_exist() {
		bool flag = false;
		for (int i = 0; i < channel_num;i++) {
			if (Controller_state[i]==0) {
				flag = true;
				break;
			}
		}
		return flag;
	}

	bool NVM_PHY_ONFI_NVDDR2::Route_reservation(flash_channel_ID_type channelID, flash_chip_ID_type chipID, std::vector<int>& path, int& controller_no) {//路径预定相关代码,传入的两个参数是目标通道ID和目标芯片ID,path存储路径预定成功时所用的路径，controller_no用于存储路径预定成功后所使用的控制器编号

		controller_no = Find_the_most_suitable_controller(channelID);				//route_reservation_controller_num存储此次用于路径预定的控制器编号
		if (controller_no == -1) {												    //该值若为-1说明找不到任何空闲的控制器，路径预定直接失败
			return false;
		}
		std::vector<std::pair<int, int>> Passed_coordinates;						//用于记录走过的坐标，防止路径预定形成环，第一个元素是channel的编号，第二个是chip编号
		int current_channel;														//用于记录当前路径预定到达了哪个芯片
		int current_chip;
		current_channel = controller_no;											//路径预定目前所在的芯片在选定的控制器的右边的chip
		current_chip = 0;
		Link_state[controller_no][0][0] = 1;										//与控制器相连的芯片的状态修改
		//path用于记录从控制器到目前到达的chip所经过的路径，0表示向上走，1表示向右边走，2表示向下走，3表示向左边走
		path.push_back(1);
		Passed_coordinates.push_back(std::make_pair(current_channel, current_chip));

		int*** Route_port_record = new int** [channel_count];						//Route_port_record用于记录每个chip的四个端口有没有尝试过，0表示没有用过，1表示已经使用过，-1表示根本不存在该端口                    
		for (int i = 0; i < channel_count; i++) {									//第一维度是行数
			Route_port_record[i] = new int* [chip_no_per_channel];					//第二维度是列
			for (int j = 0; j < chip_no_per_channel; j++) {							//第三维度共有4个元素表示路由器有4个端口
				Route_port_record[i][j] = new int[4];
			}
		}

		//对Route_port_record进行初始化，边缘chip有些端口是无效的，需要在初始化中初始化为无效值
		for (int i = 0; i < channel_count; i++) {
			for (int j = 0; j < chip_no_per_channel; j++) {
				for (int k = 0; k < 4; k++) {
					if (k == 0 && i == 0) {															//k==0表示是向上的端口，2d-mesh拓扑最上面一排都没有向上的链路
						Route_port_record[i][j][k] = -1;
					}
					else if (k == 1 && j == chip_no_per_channel - 1) {								//k==1表示是向右的端口，2d-mesh拓扑最右边一列都没有向右的链路
						Route_port_record[i][j][k] = -1;
					}
					else if (k == 2 && i == channel_count - 1) {										//k==2表示是向下的端口，2d-mesh拓扑最下边一排都没有向下的链路
						Route_port_record[i][j][k] = -1;
					}
					else {
						Route_port_record[i][j][k] = 0;												//其他的情况直接初始化为0，表示未被使用过
					}
				}
			}
		}

		//从下面开始预定路径
		while ((!(current_channel == channelID && current_chip == chipID)) && path.size() != 0) {    //跳出while循环的条件是要么到达目的芯片，要么回到起点
			PortDirection best_attempt_order[4];													//方向只有0，1，2，3这四种方向，表示无效方向
			get_the_best_port_attempt_order(best_attempt_order, channelID, chipID, current_channel, current_chip);									//经过函数处理之后best_attempt_order存储的是最佳的尝试顺序
			for (int i = 0; i < 4; i++) {
				int port_num = static_cast<int>(best_attempt_order[i]);								//获取对应端口方向对应的数值，如通过PortDirection::Port_UP转化为int型数值0
				if (Route_port_record[current_channel][current_chip][port_num] == -1 || Route_port_record[current_channel][current_chip][port_num] == 1) {//如果该端口不可用或者已被用过，则直接跳过这个端口
					if (i == 3) {																		//如果最后一个尝试的端口是上游过来的，则需要回溯
						int lastport = path.back();
						Passed_coordinates.pop_back();
						path.pop_back();
						int Port_number_after_inversion = port_inversion(lastport);
						update_coordinate(current_channel, current_chip, Port_number_after_inversion); //将反转的端口号作为参数
						break;
					}
					else {																			//尝试的端口是上游过来的，但仍有没有尝试的端口，需要继续尝试
						continue;
					}
				}
				if (port_inversion(port_num) == path.back()) {										//说明尝试到上游过来的端口了，直接跳过，并标记为已尝试过
					Route_port_record[current_channel][current_chip][port_num] = 1;
					if (i == 3) {																		//如果最后一个尝试的端口是上游过来的，则需要回溯
						int lastport = path.back();
						Passed_coordinates.pop_back();
						path.pop_back();
						int Port_number_after_inversion = port_inversion(lastport);
						update_coordinate(current_channel, current_chip, Port_number_after_inversion); //将反转的端口号作为参数
						break;
					}
					else {																			//尝试的端口是上游过来的，但仍有没有尝试的端口，需要继续尝试
						continue;
					}
				}
				int check_array[3];																	//check_array存储对应的用于查询路由链路状态的坐标
				Port_index_converted_to_link_index(current_channel, current_chip, port_num, check_array);	//经过这个函数处理，check_array里面存储的就是用于查询链路状态的三个坐标了

				if (Link_state[check_array[0]][check_array[1]][check_array[2]] == 0) {				//当此链路空闲且尝试的端口不是过来时候的端口，才算预定成功，如果尝试的端口是过来时候的端口就要直接回溯
					std::pair<int, int> temporary_updated_coordinate = get_updated_coordinate(current_channel, current_chip, port_num);		//获取更新后的坐标
					if (temporary_updated_coordinate.second < 0) {																			//防止路径预定到别的控制器上了
						Route_port_record[current_channel][current_chip][port_num] = 1;
						if (i == 3) {
							int lastport = path.back();
							Passed_coordinates.pop_back();
							path.pop_back();
							int Port_number_after_inversion = port_inversion(lastport);
							update_coordinate(current_channel, current_chip, Port_number_after_inversion); //将反转的端口号作为参数
							break;
						}
						else {
							continue;
						}
					}
					if (whether_a_ring_is_formed(Passed_coordinates, temporary_updated_coordinate)) {//函数进入这里说明如果使用port_num的端口预定路径将会形成环路
						Route_port_record[current_channel][current_chip][port_num] = 1;
						if (i == 3) {
							int lastport = path.back();
							Passed_coordinates.pop_back();
							path.pop_back();
							int Port_number_after_inversion = port_inversion(lastport);
							update_coordinate(current_channel, current_chip, Port_number_after_inversion); //将反转的端口号作为参数
							break;
						}
						else {
							continue;
						}
					}
					Route_port_record[current_channel][current_chip][port_num] = 1;					//表示该端口已经尝试过，以后不能再从这个端口尝试了
					path.push_back(port_num);														//将相应路径加入到path数组中		
					update_coordinate(current_channel, current_chip, port_num);						//更新current_channel及current_chip的值
					Passed_coordinates.push_back(std::make_pair(current_channel, current_chip));	//更新完坐标再加入到Passed_coordinates中
					break;																			//由于找到了路径，直接跳出这个for循环
				}

				else {																				//说明这个端口路径繁忙
					Route_port_record[current_channel][current_chip][port_num] = 1;
					bool flag = Backtracking_or_not(Route_port_record, current_channel, current_chip);	//flag==true说明需要回溯
					if (flag) {																		//只有当目前是最后尝试的端口，且该端口不是上游过来的端口时，代码才能走到这里
						int lastport = path.back();
						Passed_coordinates.pop_back();
						path.pop_back();
						int Port_number_after_inversion = port_inversion(lastport);
						update_coordinate(current_channel, current_chip, Port_number_after_inversion); //将反转的端口号作为参数
						break;
					}
					else {																			//不需要回溯，继续尝试别的端口就行
						continue;
					}

				}
			}
		}

		//释放Route_port_record占用的内存
		if (Route_port_record != nullptr) {
			for (int i = 0; i < channel_count; ++i) {
				if (Route_port_record[i] != nullptr) {
					for (int j = 0; j < chip_no_per_channel; ++j) {
						if (Route_port_record[i][j] != nullptr) {
							delete[] Route_port_record[i][j]; // 释放最内层的 int 数组
						}
					}
					delete[] Route_port_record[i]; // 释放中间层的 int* 数组
				}
			}
			delete[] Route_port_record; // 释放最外层的 int** 数组
		}

		// 将指针设置为 nullptr，防止悬挂指针
		Route_port_record = nullptr;

		if (path.size() == 0) {
			return false;
		}
		if (current_channel == channelID && current_chip == chipID) {					//如果到达了目标芯片，表示路径预定成功了
			Set_link_status_based_on_path(path, Link_state, controller_no);
			return true;
		}
	}

	//查找芯片通过两个参数，1：通道id，2：对应通道上的chip的id
	inline NVM::FlashMemory::Flash_Chip* NVM_PHY_ONFI_NVDDR2::Get_chip(flash_channel_ID_type channelID, flash_chip_ID_type chipID)
	{
		return channels[channelID]->Chips[chipID];
	}

	LPA_type NVM_PHY_ONFI_NVDDR2::Get_metadata(flash_channel_ID_type channe_id, flash_chip_ID_type chip_id, flash_die_ID_type die_id, flash_plane_ID_type plane_id, flash_block_ID_type block_id, flash_page_ID_type page_id)//A simplification to decrease the complexity of GC execution! The GC unit may need to know the metadata of a page to decide if a page is valid or invalid. 
	{
		return channels[channe_id]->Chips[chip_id]->Get_metadata(die_id, plane_id, block_id, page_id);
	}

	inline bool NVM_PHY_ONFI_NVDDR2::HasSuspendedCommand(NVM::FlashMemory::Flash_Chip* chip)
	{
		return bookKeepingTable[chip->ChannelID][chip->ChipID].HasSuspend;
	}

	inline ChipStatus NVM_PHY_ONFI_NVDDR2::GetChipStatus(NVM::FlashMemory::Flash_Chip* chip)
	{
		return bookKeepingTable[chip->ChannelID][chip->ChipID].Status;
	}

	inline sim_time_type NVM_PHY_ONFI_NVDDR2::Expected_finish_time(NVM::FlashMemory::Flash_Chip* chip)
	{
		return bookKeepingTable[chip->ChannelID][chip->ChipID].Expected_command_exec_finish_time;
	}

	sim_time_type NVM_PHY_ONFI_NVDDR2::Expected_finish_time(NVM_Transaction_Flash* transaction)
	{
		return Expected_finish_time(channels[transaction->Address.ChannelID]->Chips[transaction->Address.ChipID]);
	}


	sim_time_type NVM_PHY_ONFI_NVDDR2::Expected_transfer_time(NVM_Transaction_Flash* transaction)
	{
		return NVDDR2DataInTransferTime(transaction->Data_and_metadata_size_in_byte, channels[transaction->Address.ChannelID]);
	}

	NVM_Transaction_Flash* NVM_PHY_ONFI_NVDDR2::Is_chip_busy_with_stream(NVM_Transaction_Flash* transaction)
	{
		ChipBookKeepingEntry* chipBKE = &bookKeepingTable[transaction->Address.ChannelID][transaction->Address.ChipID];
		stream_id_type stream_id = transaction->Stream_id;

		for (unsigned int die_id = 0; die_id < die_no_per_chip; die_id++) {
			for (auto& tr : chipBKE->Die_book_keeping_records[die_id].ActiveTransactions) {
				if (tr->Stream_id == stream_id) {
					return tr;
				}
			}
		}

		return NULL;
	}

	bool NVM_PHY_ONFI_NVDDR2::Is_chip_busy(NVM_Transaction_Flash* transaction)
	{
		ChipBookKeepingEntry* chipBKE = &bookKeepingTable[transaction->Address.ChannelID][transaction->Address.ChipID];
		return (chipBKE->Status != ChipStatus::IDLE);
	}

	void NVM_PHY_ONFI_NVDDR2::Change_flash_page_status_for_preconditioning(const NVM::FlashMemory::Physical_Page_Address& page_address, const LPA_type lpa)
	{
		channels[page_address.ChannelID]->Chips[page_address.ChipID]->Change_memory_status_preconditioning(&page_address, &lpa);
	}

	ChipBookKeepingEntry** NVM_PHY_ONFI_NVDDR2::get_ChipBookKeepingEntry() {																//获取ChipBookKeepingEntry
		return bookKeepingTable;
	}

	//会注册一个传输命令的事件，相关传输事务会放进dieBKE的ActiveTransactions中
	void NVM_PHY_ONFI_NVDDR2::Send_command_to_chip(std::list<NVM_Transaction_Flash*>& transaction_list)   //transaction_list中的事务都是在同一个chip上的事务
	{
		static int num = 0;
		num++;
		/*std::cout << "send command to chip execute " << num << " times " << "at "<<Simulator->Time()<<std::endl;*/

		unsigned int byte_num = transaction_list.front()->Data_and_metadata_size_in_byte;

		ONFI_Channel_NVDDR2* target_channel = channels[transaction_list.front()->Address.ChannelID];

		NVM::FlashMemory::Flash_Chip* targetChip = target_channel->Chips[transaction_list.front()->Address.ChipID];
		ChipBookKeepingEntry* chipBKE = &bookKeepingTable[transaction_list.front()->Address.ChannelID][transaction_list.front()->Address.ChipID];
		DieBookKeepingEntry* dieBKE = &chipBKE->Die_book_keeping_records[transaction_list.front()->Address.DieID];

		int tl_size = transaction_list.size();
		int as_size = dieBKE->ActiveTransactions.size();
		if (tl_size!= as_size) {
			std::cout << "problem occurs at " << Simulator->Time()<<std::endl;
			
		}
		/*If this is not a die-interleaved command execution, and the channel is already busy,
		* then something illegarl is happening*/

		//if (target_channel->GetStatus() == BusChannelStatus::BUSY && chipBKE->OngoingDieCMDTransfers.size() == 0) {  //通道繁忙且目标芯片没有正在执行的命令，说明通道上有别的芯片在执行命令
		//	PRINT_ERROR("Bus " << target_channel->ChannelID << ": starting communication on a busy flash channel!");
		//}

		sim_time_type suspendTime = 0;

		//if (!dieBKE->Free) {										 //如果进入这段函数说明当前Die上已经有事务在执行
		//	if (transaction_list.front()->SuspendRequired) {		 //当前事务需要暂停
		//		switch (dieBKE->ActiveTransactions.front()->Type) {  //检查当前正在执行的事务类型
		//		case Transaction_Type::WRITE:
		//			Stats::IssuedSuspendProgramCMD++;
		//			suspendTime = target_channel->ProgramSuspendCommandTime + targetChip->GetSuspendProgramTime();   //计算擦除暂停所需时间
		//			break;
		//		case Transaction_Type::ERASE:
		//			Stats::IssuedSuspendEraseCMD++;
		//			suspendTime = target_channel->EraseSuspendCommandTime + targetChip->GetSuspendEraseTime();
		//			break;
		//		default:
		//			/*std::cout << "up" << std::endl;*/
		//			PRINT_ERROR("Read suspension is not supported!")
		//		}
		//		targetChip->Suspend(transaction_list.front()->Address.DieID);  //修改的是die相关的信息
		//		dieBKE->PrepareSuspend();									   //修改的是dieBKE相关的信息
		//		if (chipBKE->OngoingDieCMDTransfers.size()) {
		//			chipBKE->PrepareSuspend();
		//		}
		//	}
		//	else {
		//		/*std::cout << "down" << std::endl;*/
		//		PRINT_ERROR("Read suspension is not supported!")
		//	}
		//}

		/*dieBKE->Free = false;*/															  //提前放到issue_command_to_chip函数中

		/*dieBKE->ActiveCommand = new NVM::FlashMemory::Flash_Command(); */               //代码移动到issue_command_to_chip上，创建一个新命令，改变了dieBKE上原本执行的命令，并将dieBKE命令改成创建的命令

		//for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_list.begin();   //将command的一些信息放进了dieBKE中
		//	it != transaction_list.end(); it++) {
			/*dieBKE->ActiveTransactions.push_back(*it);	*/							  //被移动到issue_command_to_chip中
			/*dieBKE->ActiveCommand->Address.push_back((*it)->Address);
			NVM::FlashMemory::PageMetadata metadata;
			metadata.LPA = (*it)->LPA;
			dieBKE->ActiveCommand->Meta_data.push_back(metadata);*/
		/*}*/

		switch (transaction_list.front()->Type) {
		case Transaction_Type::READ:
			if (transaction_list.size() == 1) {
				Stats::IssuedReadCMD++;
				dieBKE->ActiveCommand->CommandCode = CMD_READ_PAGE;
				DEBUG("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << transaction_list.front()->Address.DieID << ": Sending read command to chip for LPA: " << transaction_list.front()->LPA)
			}
			else {
				int static num = 0;
				num++;
				std::cout << "MultiplaneReadCMD is " << num <<" at "<<Simulator->Time()<< std::endl;
				Stats::IssuedMultiplaneReadCMD++;
				dieBKE->ActiveCommand->CommandCode = CMD_READ_PAGE_MULTIPLANE;
				DEBUG("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << transaction_list.front()->Address.DieID << ": Sending multi-plane read command to chip for LPA: " << transaction_list.front()->LPA)
			}

			for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_list.begin();
				it != transaction_list.end(); it++) {
				(*it)->STAT_transfer_time += target_channel->ReadCommandTime[transaction_list.size()];   //将命令传输时间存储到对应的事务对象里面
			}
			if (chipBKE->OngoingDieCMDTransfers.size() == 0) {					//检查正在进行的命令传输数量是否为0
				targetChip->StartCMDXfer();										//将当前时间设置为该chip的lastTransferStart
				chipBKE->Status = ChipStatus::CMD_IN;							//chipBKE状态设置为命令传输中
				chipBKE->Last_transfer_finish_time = Simulator->Time() + suspendTime + target_channel->ReadCommandTime[transaction_list.size()];
				Simulator->Register_sim_event(Simulator->Time() + suspendTime + target_channel->ReadCommandTime[transaction_list.size()], this,
					dieBKE, (int)NVDDR2_SimEventType::READ_CMD_ADDR_TRANSFERRED);
			}
			else {
				dieBKE->DieInterleavedTime = suspendTime + target_channel->ReadCommandTime[transaction_list.size()];
				chipBKE->Last_transfer_finish_time += suspendTime + target_channel->ReadCommandTime[transaction_list.size()];
			}
			chipBKE->OngoingDieCMDTransfers.push(dieBKE);

			dieBKE->Expected_finish_time = chipBKE->Last_transfer_finish_time + targetChip->Get_command_execution_latency(dieBKE->ActiveCommand->CommandCode, dieBKE->ActiveCommand->Address[0].PageID);
			if (chipBKE->Expected_command_exec_finish_time < dieBKE->Expected_finish_time) {
				chipBKE->Expected_command_exec_finish_time = dieBKE->Expected_finish_time;
			}
			break;
		case Transaction_Type::WRITE:
			if (((NVM_Transaction_Flash_WR*)transaction_list.front())->ExecutionMode == WriteExecutionModeType::SIMPLE) {//WriteExecutionModeType分为两种，一种是普通写，另一种是copyback写
				if (transaction_list.size() == 1) {
					Stats::IssuedProgramCMD++;
					dieBKE->ActiveCommand->CommandCode = CMD_PROGRAM_PAGE;
					DEBUG("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << transaction_list.front()->Address.DieID << ": Sending program command to chip for LPA: " << transaction_list.front()->LPA)
				}
				else {
					Stats::IssuedMultiplaneProgramCMD++;
					dieBKE->ActiveCommand->CommandCode = CMD_PROGRAM_PAGE_MULTIPLANE;
					DEBUG("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << transaction_list.front()->Address.DieID << ": Sending multi-plane program command to chip for LPA: " << transaction_list.front()->LPA)
				}

				sim_time_type data_transfer_time = 0;

				for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_list.begin();
					it != transaction_list.end(); it++) {
					(*it)->STAT_transfer_time += target_channel->ProgramCommandTime[transaction_list.size()] + NVDDR2DataInTransferTime((*it)->Data_and_metadata_size_in_byte, target_channel);    //累加了命令传输时间也包括了数据传输时间
					data_transfer_time += NVDDR2DataInTransferTime((*it)->Data_and_metadata_size_in_byte, target_channel);      //写请求的具体数据传输时间
				}
				if (chipBKE->OngoingDieCMDTransfers.size() == 0) {
					targetChip->StartCMDDataInXfer();
					chipBKE->Status = ChipStatus::CMD_DATA_IN;
					chipBKE->Last_transfer_finish_time = Simulator->Time() + suspendTime + target_channel->ProgramCommandTime[transaction_list.size()] + data_transfer_time;
					Simulator->Register_sim_event(Simulator->Time() + suspendTime + target_channel->ProgramCommandTime[transaction_list.size()] + data_transfer_time,
						this, dieBKE, (int)NVDDR2_SimEventType::PROGRAM_CMD_ADDR_DATA_TRANSFERRED);
				}
				else {//程序进行到这里说明有正在执行的命令，对于时间的计算需要计算上挂起操作
					dieBKE->DieInterleavedTime = suspendTime + target_channel->ProgramCommandTime[transaction_list.size()] + data_transfer_time;
					chipBKE->Last_transfer_finish_time += suspendTime + target_channel->ProgramCommandTime[transaction_list.size()] + data_transfer_time;
				}
				chipBKE->OngoingDieCMDTransfers.push(dieBKE);

				dieBKE->Expected_finish_time = chipBKE->Last_transfer_finish_time + targetChip->Get_command_execution_latency(dieBKE->ActiveCommand->CommandCode, dieBKE->ActiveCommand->Address[0].PageID);
				if (chipBKE->Expected_command_exec_finish_time < dieBKE->Expected_finish_time) {
					chipBKE->Expected_command_exec_finish_time = dieBKE->Expected_finish_time;
				}
			}
			else {
				//Copyback write for GC

				if (transaction_list.size() == 1) {
					Stats::IssuedCopybackReadCMD++;
					dieBKE->ActiveCommand->CommandCode = CMD_READ_PAGE_COPYBACK;
				}
				else {
					Stats::IssuedMultiplaneCopybackProgramCMD++;
					dieBKE->ActiveCommand->CommandCode = CMD_READ_PAGE_COPYBACK_MULTIPLANE;
				}

				for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_list.begin();
					it != transaction_list.end(); it++) {
					(*it)->STAT_transfer_time += target_channel->ReadCommandTime[transaction_list.size()];
				}
				if (chipBKE->OngoingDieCMDTransfers.size() == 0) {
					targetChip->StartCMDXfer();
					chipBKE->Status = ChipStatus::CMD_IN;
					chipBKE->Last_transfer_finish_time = Simulator->Time() + suspendTime + target_channel->ReadCommandTime[transaction_list.size()];
					Simulator->Register_sim_event(Simulator->Time() + suspendTime + target_channel->ReadCommandTime[transaction_list.size()], this,
						dieBKE, (int)NVDDR2_SimEventType::READ_CMD_ADDR_TRANSFERRED);
				}
				else {
					dieBKE->DieInterleavedTime = suspendTime + target_channel->ReadCommandTime[transaction_list.size()];
					chipBKE->Last_transfer_finish_time += suspendTime + target_channel->ReadCommandTime[transaction_list.size()];
				}
				chipBKE->OngoingDieCMDTransfers.push(dieBKE);

				dieBKE->Expected_finish_time = chipBKE->Last_transfer_finish_time + targetChip->Get_command_execution_latency(dieBKE->ActiveCommand->CommandCode, dieBKE->ActiveCommand->Address[0].PageID);
				if (chipBKE->Expected_command_exec_finish_time < dieBKE->Expected_finish_time) {
					chipBKE->Expected_command_exec_finish_time = dieBKE->Expected_finish_time;
				}
			}
			break;
		case Transaction_Type::ERASE:
			//DEBUG2("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << transaction_list.front()->Address.DieID << ": Sending erase command to chip")
			if (transaction_list.size() == 1) {
				Stats::IssuedEraseCMD++;
				dieBKE->ActiveCommand->CommandCode = CMD_ERASE_BLOCK;
			}
			else {
				Stats::IssuedMultiplaneEraseCMD++;
				dieBKE->ActiveCommand->CommandCode = CMD_ERASE_BLOCK_MULTIPLANE;
			}

			for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_list.begin();
				it != transaction_list.end(); it++) {
				(*it)->STAT_transfer_time += target_channel->EraseCommandTime[transaction_list.size()];
			}
			if (chipBKE->OngoingDieCMDTransfers.size() == 0) {
				targetChip->StartCMDXfer();
				chipBKE->Status = ChipStatus::CMD_IN;
				chipBKE->Last_transfer_finish_time = Simulator->Time() + suspendTime + target_channel->EraseCommandTime[transaction_list.size()];
				Simulator->Register_sim_event(Simulator->Time() + suspendTime + target_channel->EraseCommandTime[transaction_list.size()],
					this, dieBKE, (int)NVDDR2_SimEventType::ERASE_SETUP_COMPLETED);
			}
			else {
				dieBKE->DieInterleavedTime = suspendTime + target_channel->EraseCommandTime[transaction_list.size()];
				chipBKE->Last_transfer_finish_time += suspendTime + target_channel->EraseCommandTime[transaction_list.size()];
			}
			chipBKE->OngoingDieCMDTransfers.push(dieBKE);

			dieBKE->Expected_finish_time = chipBKE->Last_transfer_finish_time + targetChip->Get_command_execution_latency(dieBKE->ActiveCommand->CommandCode, dieBKE->ActiveCommand->Address[0].PageID);
			if (chipBKE->Expected_command_exec_finish_time < dieBKE->Expected_finish_time) {
				chipBKE->Expected_command_exec_finish_time = dieBKE->Expected_finish_time;
			}
			break;
		default:
			throw std::invalid_argument("NVM_PHY_ONFI_NVDDR2: Unhandled event specified!");
		}

		target_channel->SetStatus(BusChannelStatus::BUSY, targetChip);
	}

	void NVM_PHY_ONFI_NVDDR2::Change_memory_status_preconditioning(const NVM::NVM_Memory_Address* address, const void* status_info)
	{
		channels[((NVM::FlashMemory::Physical_Page_Address*)address)->ChannelID]->Chips[((NVM::FlashMemory::Physical_Page_Address*)address)->ChipID]->Change_memory_status_preconditioning(address, status_info);
	}

	void copy_read_data_to_transaction(NVM_Transaction_Flash_RD* read_transaction, NVM::FlashMemory::Flash_Command* command)
	{
		int i = 0;
		for (auto& address : command->Address) {
			if (address.PlaneID == read_transaction->Address.PlaneID) {//因为传入的都是同一个diebke，所以只要保证PlaneID相同就行
				read_transaction->LPA = command->Meta_data[i].LPA;
			}
			i++;
		}
	}

	void NVM_PHY_ONFI_NVDDR2::Execute_simulator_event(MQSimEngine::Sim_Event* ev)
	{
		DieBookKeepingEntry* dieBKE = (DieBookKeepingEntry*)ev->Parameters;								//先获取die的相关记录
		flash_channel_ID_type channel_id = dieBKE->ActiveTransactions.front()->Address.ChannelID;		//再通过dieBKE获取channel的id
		ONFI_Channel_NVDDR2* targetChannel = channels[channel_id];
		NVM::FlashMemory::Flash_Chip* targetChip = targetChannel->Chips[dieBKE->ActiveTransactions.front()->Address.ChipID];  //获取事务的目标chip
		ChipBookKeepingEntry* chipBKE = &bookKeepingTable[channel_id][targetChip->ChipID];

		switch ((NVDDR2_SimEventType)ev->Type) {
		case NVDDR2_SimEventType::ROUTE_RESERVATION:					//代码执行到这个地方说明路径预定的时间已经计算上了
			Send_command_to_chip(dieBKE->temporary_transactions);
			break;
		case NVDDR2_SimEventType::READ_CMD_ADDR_TRANSFERRED:     //读命令传输阶段
			//DEBUG2("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << dieBKE->ActiveTransactions.front()->Address.DieID << ": READ_CMD_ADDR_TRANSFERRED ")
			targetChip->EndCMDXfer(dieBKE->ActiveCommand);		 //走到这个地方的时候，命令传输到了chip，但是命令并没有在die上执行
			for (auto tr : dieBKE->ActiveTransactions) {
				tr->STAT_execution_time = dieBKE->Expected_finish_time - Simulator->Time();
			}
			chipBKE->OngoingDieCMDTransfers.pop();				//将命令移出队列
			chipBKE->No_of_active_dies++;						//可用die增加
			if (chipBKE->OngoingDieCMDTransfers.size() > 0) {	//检查是否还有正在进行的die命令传输
				perform_interleaved_cmd_data_transfer(targetChip, chipBKE->OngoingDieCMDTransfers.front());		//chip上还有未完成的命令传输通过这个函数注册新的命令传输事件
				return;
			}
			else {
				chipBKE->Status = ChipStatus::READING;			//如果没有正在进行的命令传输，将芯片设置为读取状态
				targetChannel->SetStatus(BusChannelStatus::IDLE, targetChip);			//总线设置为空闲状态
			}
			break;
		case NVDDR2_SimEventType::ERASE_SETUP_COMPLETED:
			//DEBUG2("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << dieBKE->ActiveTransactions.front()->Address.DieID << ": ERASE_SETUP_COMPLETED ")
			targetChip->EndCMDXfer(dieBKE->ActiveCommand);
			for (auto& tr : dieBKE->ActiveTransactions) {
				tr->STAT_execution_time = dieBKE->Expected_finish_time - Simulator->Time();
			}
			chipBKE->OngoingDieCMDTransfers.pop();
			chipBKE->No_of_active_dies++;
			if (chipBKE->OngoingDieCMDTransfers.size() > 0) {
				perform_interleaved_cmd_data_transfer(targetChip, chipBKE->OngoingDieCMDTransfers.front());
				return;
			}
			else {
				chipBKE->Status = ChipStatus::ERASING;
				targetChannel->SetStatus(BusChannelStatus::IDLE, targetChip);
			}
			break;
		case NVDDR2_SimEventType::PROGRAM_CMD_ADDR_DATA_TRANSFERRED:      //写指令命令及数据传输阶段，执行到这里的时候，已经把命令和数据传输到目标chip上了
		case NVDDR2_SimEventType::PROGRAM_COPYBACK_CMD_ADDR_TRANSFERRED:
			//DEBUG2("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << dieBKE->ActiveTransactions.front()->Address.DieID <<  ": PROGRAM_CMD_ADDR_DATA_TRANSFERRED " )
			dieBKE->temporary_transactions.clear();                                 //清除用于暂存向send_command_to_chip传递参数的temporary_transactions
			Controller_state[dieBKE->ActiveCommand->controller_number] = 0;			//将该事务所负责的控制器设置为空闲
			Set_link_status_to_idle_based_on_the_path(dieBKE->ActiveCommand->path, Link_state, dieBKE->ActiveCommand->controller_number);

			/*std::cout << channel_id << " " << targetChip->ChipID << " " << "WRITE " << "COMPLETED AT " << Simulator->Time() << std::endl;*/

			targetChip->EndCMDDataInXfer(dieBKE->ActiveCommand);
			for (auto& tr : dieBKE->ActiveTransactions) {
				tr->STAT_execution_time = dieBKE->Expected_finish_time - Simulator->Time();
			}
			chipBKE->OngoingDieCMDTransfers.pop();
			chipBKE->No_of_active_dies++;
			if (chipBKE->OngoingDieCMDTransfers.size() > 0)
			{
				perform_interleaved_cmd_data_transfer(targetChip, chipBKE->OngoingDieCMDTransfers.front());
				return;
			}
			else {
				chipBKE->Status = ChipStatus::WRITING;
				targetChannel->SetStatus(BusChannelStatus::IDLE, targetChip);
			}
			break;
		case NVDDR2_SimEventType::READ_DATA_TRANSFERRED:							//将读出的数据传回控制器的阶段
			//DEBUG2("Chip " << targetChip->ChannelID << ", " << targetChip->ChipID << ", " << dieBKE->ActiveTransactions.front()->Address.DieID << ": READ_DATA_TRANSFERRED ")
			targetChip->EndDataOutXfer(dieBKE->ActiveCommand);
			if (Simulator->Time()==73203) {
				std::cout << "first transfer ends" << std::endl;
			}
			/*dieBKE->temporary_transactions.clear();*/                                 //清除用于暂存向send_command_to_chip传递参数的temporary_transactions

			//Set_link_status_to_idle_based_on_the_path(dieBKE->ActiveCommand->path, Link_state, dieBKE->ActiveCommand->controller_number);
			//Controller_state[dieBKE->ActiveCommand->controller_number] = 0;

			copy_read_data_to_transaction((NVM_Transaction_Flash_RD*)dieBKE->ActiveTransfer, dieBKE->ActiveCommand);							//将正在执行的命令的元数据赋值到正在进行的传输事务中
#if 0
			if (tr->ExecutionMode != ExecutionModeType::COPYBACK)
#endif
				//if (Simulator->Time()==3368098) {
				//	std::cout<<"wow"<<std::endl;
				//}
			broadcastTransactionServicedSignal(dieBKE->ActiveTransfer);	//经过这个函数处理，dieBKE中的ActiveTransactions以及ActiveTransfer就已经被清空了										
																			//调用GC and WL Unit，Data Cache Manager及Address Mapping Unit这三个的handle_transaction_serviced_signal_from_PHY函数		
			
			for (std::list<NVM_Transaction_Flash*>::iterator it = dieBKE->ActiveTransactions.begin();					//在ActiveTransactions双向链表中删除当前事务（即ActiveTransfer）
				it != dieBKE->ActiveTransactions.end(); it++) {
				if ((*it) == dieBKE->ActiveTransfer) {
					dieBKE->ActiveTransactions.erase(it);
					break;
				}
			}

			for (std::list<NVM_Transaction_Flash*>::iterator it = dieBKE->temporary_transactions.begin();					
				it != dieBKE->temporary_transactions.end(); it++) {
				if ((*it) == dieBKE->ActiveTransfer) {
					dieBKE->temporary_transactions.erase(it);
					break;
				}
			}
			
			dieBKE->ActiveTransfer = NULL;
			//////////////////////////////////////////////////////////////////////////////////////
			//if (channel_id==4&& targetChip->ChipID==1&&Simulator->Time()>30000) {
			//	std::cout << "ActiveTransfer were clear at " << Simulator->Time() << std::endl;
			//}

			if (dieBKE->ActiveTransactions.size() == 0) {																	//当传输命令全部结束的时候，再将使用的链路以及控制器设置为空闲
				Set_link_status_to_idle_based_on_the_path(dieBKE->ActiveCommand->path, Link_state, dieBKE->ActiveCommand->controller_number);
				Controller_state[dieBKE->ActiveCommand->controller_number] = 0;
				chipBKE->WaitingReadTXCount--;			//由于模拟器对WaitingReadTXCount的判断并不够精准，我们将一个多平面读命令视为一个WaitingReadTXCount，两个读完成了再自减
				dieBKE->ClearCommand();
			}
			else if(dieBKE->ActiveCommand->CommandCode == CMD_READ_PAGE_MULTIPLANE && dieBKE->ActiveTransactions.size()>0) { //用于处理多平面命令的第二个请求
				transfer_read_data_from_chip(chipBKE, dieBKE, dieBKE->ActiveTransactions.front(), dieBKE->ActiveTransactions);
			}

			/*chipBKE->WaitingReadTXCount--;*/

			if (chipBKE->No_of_active_dies == 0) {
				if (chipBKE->WaitingReadTXCount == 0) {
					chipBKE->Status = ChipStatus::IDLE;
				}
				else {
					chipBKE->Status = ChipStatus::WAIT_FOR_DATA_OUT;
				}
			}
			if (chipBKE->Status == ChipStatus::IDLE) {
				if (dieBKE->Suspended) {
					send_resume_command_to_chip(targetChip, chipBKE);													//有挂起的操作则恢复挂起操作
				}
			}
			targetChannel->SetStatus(BusChannelStatus::IDLE, targetChip);												//无论有没有恢复挂起操作，下面的通道都应该恢复IDLE状态，因为挂起的操作是在chip上进行的，不会影响通道的状态
			break;
		default:
			PRINT_ERROR("Unknown simulation event specified for NVM_PHY_ONFI_NVDDR2!")
		}

		/* Copyback requests are prioritized over other type of requests since they need very short transfer time.
		In addition, they are just used for GC purpose. */

		//检查对应模拟事件是否还有等待的copyback操作，有的话就执行命令并更新状态及统计信息
		std::vector<int> WaitingCopybackWrites_path;
		std::vector<int> WaitingMappingRead_TX_path;
		std::vector<int> WaitingReadTX_path;
		std::vector<int> WaitingGCRead_TX_path;

		int WaitingCopybackWrites_controller=100;
		int WaitingMappingRead_TX_controller = 100;
		int WaitingReadTX_controller = 100;
		int WaitingGCRead_TX_controller = 100;

		if (WaitingCopybackWrites[channel_id].size() > 0) {
			DieBookKeepingEntry* waitingBKE = WaitingCopybackWrites[channel_id].front();
			WaitingCopybackWrites_path = waitingBKE->ActiveCommand->path;
			WaitingCopybackWrites_controller = waitingBKE->ActiveCommand->controller_number;
		}
		if (WaitingMappingRead_TX[channel_id].size() > 0) {
			NVM_Transaction_Flash_RD* waitingTR = (NVM_Transaction_Flash_RD*)WaitingMappingRead_TX[channel_id].front();
			DieBookKeepingEntry waitingBKE = bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID];
			WaitingMappingRead_TX_path = waitingBKE.ActiveCommand->path;
			WaitingMappingRead_TX_controller = waitingBKE.ActiveCommand->controller_number;
		}
		if (WaitingReadTX[channel_id].size() > 0) {
			NVM_Transaction_Flash_RD* waitingTR = (NVM_Transaction_Flash_RD*)WaitingReadTX[channel_id].front();
			DieBookKeepingEntry waitingBKE = bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID];
			WaitingReadTX_path = waitingBKE.ActiveCommand->path;
			WaitingReadTX_controller = waitingBKE.ActiveCommand->controller_number;
		}
		if (WaitingGCRead_TX[channel_id].size() > 0) {
			NVM_Transaction_Flash_RD* waitingTR = (NVM_Transaction_Flash_RD*)WaitingGCRead_TX[channel_id].front();
			DieBookKeepingEntry waitingBKE = bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID];
			WaitingGCRead_TX_path = waitingBKE.ActiveCommand->path;
			WaitingGCRead_TX_controller = waitingBKE.ActiveCommand->controller_number;
		}


		if (WaitingCopybackWrites[channel_id].size() > 0) {
			DieBookKeepingEntry* waitingBKE = WaitingCopybackWrites[channel_id].front();
			targetChip = channels[channel_id]->Chips[waitingBKE->ActiveTransactions.front()->Address.ChipID];
			ChipBookKeepingEntry* waitingChipBKE = &bookKeepingTable[channel_id][targetChip->ChipID];
			if (waitingBKE->ActiveTransactions.size() > 1) {											//有两种copyback的方式，根据ActiveTransactions的大小判断是不是MULTIPLANE指令
				Stats::IssuedMultiplaneCopybackProgramCMD++;
				waitingBKE->ActiveCommand->CommandCode = CMD_PROGRAM_PAGE_COPYBACK_MULTIPLANE;
			}
			else {
				Stats::IssuedCopybackProgramCMD++;
				waitingBKE->ActiveCommand->CommandCode = CMD_PROGRAM_PAGE_COPYBACK;
			}
			targetChip->StartCMDXfer();
			waitingChipBKE->Status = ChipStatus::CMD_IN;
			Simulator->Register_sim_event(Simulator->Time() + this->channels[channel_id]->ProgramCommandTime[waitingBKE->ActiveTransactions.size()],   //注册相应的copyback事件
				this, waitingBKE, (int)NVDDR2_SimEventType::PROGRAM_COPYBACK_CMD_ADDR_TRANSFERRED);
			waitingChipBKE->OngoingDieCMDTransfers.push(waitingBKE);

			waitingBKE->Expected_finish_time = Simulator->Time() + this->channels[channel_id]->ProgramCommandTime[waitingBKE->ActiveTransactions.size()]
				+ targetChip->Get_command_execution_latency(waitingBKE->ActiveCommand->CommandCode, waitingBKE->ActiveCommand->Address[0].PageID);
			if (waitingChipBKE->Expected_command_exec_finish_time < waitingBKE->Expected_finish_time) {
				waitingChipBKE->Expected_command_exec_finish_time = waitingBKE->Expected_finish_time;
			}

			WaitingCopybackWrites[channel_id].pop_front();
			channels[channel_id]->SetStatus(BusChannelStatus::BUSY, targetChip);                  //channel的状态设置成busy

			return;
		}

		else if (WaitingMappingRead_TX[channel_id].size() > 0) {								  //检查是否有映射表读事务
			NVM_Transaction_Flash_RD* waitingTR = (NVM_Transaction_Flash_RD*)WaitingMappingRead_TX[channel_id].front();
			WaitingMappingRead_TX[channel_id].pop_front();
			transfer_read_data_from_chip(&bookKeepingTable[channel_id][waitingTR->Address.ChipID],
				&(bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID]), waitingTR, bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID].ActiveTransactions);

			return;
		}
		else if (WaitingReadTX[channel_id].size() > 0) {
			NVM_Transaction_Flash_RD* waitingTR = (NVM_Transaction_Flash_RD*)WaitingReadTX[channel_id].front();
			WaitingReadTX[channel_id].pop_front();
			transfer_read_data_from_chip(&bookKeepingTable[channel_id][waitingTR->Address.ChipID],
				&(bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID]), waitingTR, bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID].ActiveTransactions);

			return;
		}
		else if (WaitingGCRead_TX[channel_id].size() > 0) {
			NVM_Transaction_Flash_RD* waitingTR = (NVM_Transaction_Flash_RD*)WaitingGCRead_TX[channel_id].front();
			WaitingGCRead_TX[channel_id].pop_front();
			transfer_read_data_from_chip(&bookKeepingTable[channel_id][waitingTR->Address.ChipID],
				&(bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID]), waitingTR, bookKeepingTable[channel_id][waitingTR->Address.ChipID].Die_book_keeping_records[waitingTR->Address.DieID].ActiveTransactions);
			return;
		}



		//If the execution reaches here, then the bus channel became idle

		broadcastChannelIdleSignal(channel_id);
	}


	//这个函数就是chip已经把他该干的事情干完后的一些处理，如果是读操作，说明chip已经把数据读好了，等着传给控制器。如果是写操作，此时已经进行到数据写入到chip中了
	inline void NVM_PHY_ONFI_NVDDR2::handle_ready_signal_from_chip(NVM::FlashMemory::Flash_Chip* chip, NVM::FlashMemory::Flash_Command* command)
	{
		ChipBookKeepingEntry* chipBKE = &_my_instance->bookKeepingTable[chip->ChannelID][chip->ChipID];
		DieBookKeepingEntry* dieBKE = &(chipBKE->Die_book_keeping_records[command->Address[0].DieID]);

		switch (command->CommandCode)
		{
		case CMD_READ_PAGE:
		case CMD_READ_PAGE_MULTIPLANE:

		//DEBUG("Chip " << chip->ChannelID << ", " << chip->ChipID << ": finished  read command")
		//	chipBKE->No_of_active_dies--;
		//if (chipBKE->No_of_active_dies == 0)//After finishing the last command, the chip state is changed
		//	chipBKE->Status = ChipStatus::WAIT_FOR_DATA_OUT;                                      //如果所有die都完成了命令，则更改芯片的状态
		//for (std::list<NVM_Transaction_Flash*>::iterator it = dieBKE->ActiveTransactions.begin();
		//	it != dieBKE->ActiveTransactions.end(); it++)
		//{
		//	chipBKE->WaitingReadTXCount++;														  //等待读取的数量增加
		//	if (_my_instance->channels[chip->ChannelID]->GetStatus() == BusChannelStatus::IDLE)   //检查通道的状态
		//		_my_instance->transfer_read_data_from_chip(chipBKE, dieBKE, (*it));				  //将数据从chip传输到控制器，注册NVDDR2_SimEventType::READ_DATA_TRANSFERRED事件，将通道设置为busy状态
		//	else
		//	{
		//		switch (dieBKE->ActiveTransactions.front()->Source)
		//		{
		//		case Transaction_Source_Type::CACHE:
		//		case Transaction_Source_Type::USERIO:
		//			_my_instance->WaitingReadTX[chip->ChannelID].push_back((*it));
		//			if (Simulator->Time()==60915) {
		//				std::cout << "wow" << std::endl;
		//			}
		//			std::cout << "WaitingReadTX push_back at " << Simulator->Time() << std::endl;
		//			break;
		//		case Transaction_Source_Type::GC_WL:
		//			_my_instance->WaitingGCRead_TX[chip->ChannelID].push_back((*it));
		//			break;
		//		case Transaction_Source_Type::MAPPING:
		//			_my_instance->WaitingMappingRead_TX[chip->ChannelID].push_back((*it));
		//			std::cout << "WaitingMappingRead_TX push_back at " << Simulator->Time() << std::endl;
		//			break;
		//		}
		//	}
		//}
		//break;

		DEBUG("Chip " << chip->ChannelID << ", " << chip->ChipID << ": finished  read command")
			chipBKE->No_of_active_dies--;
		if (chipBKE->No_of_active_dies == 0)//After finishing the last command, the chip state is changed
			chipBKE->Status = ChipStatus::WAIT_FOR_DATA_OUT;
			chipBKE->WaitingReadTXCount++;																														//等待读取的数量增加
			if (dieBKE->ActiveTransactions.size() == 1) {																										//检查通道的状态
				_my_instance->transfer_read_data_from_chip(chipBKE, dieBKE, (*dieBKE->ActiveTransactions.begin()), dieBKE->ActiveTransactions);				  //将数据从chip传输到控制器，注册NVDDR2_SimEventType::READ_DATA_TRANSFERRED事件，将通道设置为busy状态
			}
			else if(dieBKE->ActiveTransactions.size() > 1)
			{
				_my_instance->transfer_read_data_from_chip(chipBKE, dieBKE, (*dieBKE->ActiveTransactions.begin()), dieBKE->ActiveTransactions);
				/*switch (dieBKE->ActiveTransactions.front()->Source)
				{
					case Transaction_Source_Type::CACHE:
					case Transaction_Source_Type::USERIO:
						_my_instance->WaitingReadTX[chip->ChannelID].push_back(dieBKE->ActiveTransactions.back());
						if (Simulator->Time() == 60915) {
							std::cout << "wow" << std::endl;
						}
						std::cout << "WaitingReadTX push_back at " << Simulator->Time() << std::endl;
						break;
					case Transaction_Source_Type::GC_WL:
						_my_instance->WaitingGCRead_TX[chip->ChannelID].push_back(dieBKE->ActiveTransactions.back());
						std::cout << "WaitingGCRead_TX push_back at " << Simulator->Time() << std::endl;
						break;
					case Transaction_Source_Type::MAPPING:
						_my_instance->WaitingMappingRead_TX[chip->ChannelID].push_back(dieBKE->ActiveTransactions.back());
						std::cout << "WaitingMappingRead_TX push_back at " << Simulator->Time() << std::endl;
						break;
				}*/
			}
		break;

		//目前来说，下面这两种情况完全不会执行
		case CMD_READ_PAGE_COPYBACK:
		case CMD_READ_PAGE_COPYBACK_MULTIPLANE:
			chipBKE->No_of_active_dies--;
			if (chipBKE->No_of_active_dies == 0)
				chipBKE->Status = ChipStatus::WAIT_FOR_COPYBACK_CMD;
			if (_my_instance->channels[chip->ChannelID]->GetStatus() == BusChannelStatus::IDLE)
			{
				if (dieBKE->ActiveTransactions.size() > 1)
				{
					Stats::IssuedMultiplaneCopybackProgramCMD++;
					dieBKE->ActiveCommand->CommandCode = CMD_PROGRAM_PAGE_COPYBACK_MULTIPLANE;
				}
				else
				{
					Stats::IssuedCopybackProgramCMD++;
					dieBKE->ActiveCommand->CommandCode = CMD_PROGRAM_PAGE_COPYBACK;
				}

				for (std::list<NVM_Transaction_Flash*>::iterator it = dieBKE->ActiveTransactions.begin();
					it != dieBKE->ActiveTransactions.end(); it++)
				{
					(*it)->STAT_transfer_time += _my_instance->channels[chip->ChannelID]->ProgramCommandTime[dieBKE->ActiveTransactions.size()];
				}
				chip->StartCMDXfer();
				chipBKE->Status = ChipStatus::CMD_IN;
				Simulator->Register_sim_event(Simulator->Time() + _my_instance->channels[chip->ChannelID]->ProgramCommandTime[dieBKE->ActiveTransactions.size()],
					_my_instance, dieBKE, (int)NVDDR2_SimEventType::PROGRAM_COPYBACK_CMD_ADDR_TRANSFERRED);
				chipBKE->OngoingDieCMDTransfers.push(dieBKE);
				_my_instance->channels[chip->ChannelID]->SetStatus(BusChannelStatus::BUSY, chip);

				dieBKE->Expected_finish_time = Simulator->Time() + _my_instance->channels[chip->ChannelID]->ProgramCommandTime[dieBKE->ActiveTransactions.size()]
					+ chip->Get_command_execution_latency(dieBKE->ActiveCommand->CommandCode, dieBKE->ActiveCommand->Address[0].PageID);
				if (chipBKE->Expected_command_exec_finish_time < dieBKE->Expected_finish_time)
					chipBKE->Expected_command_exec_finish_time = dieBKE->Expected_finish_time;
#if 0	
				//Copyback data should be read out in order to get rid of bit error propagation
				Simulator->RegisterEvent(Simulator->Time() + channels[targetChip->ChannelID]->ProgramCommandTime + NVDDR2DataOutTransferTime(targetTransaction->SizeInByte, channels[targetChip->ChannelID]),
					this, targetTransaction, (int)NVDDR2_SimEventType::READ_DATA_TRANSFERRED);
				targetTransaction->STAT_TransferTime += NVDDR2DataOutTransferTime(targetTransaction->SizeInByte, channels[targetChip->ChannelID]);
#endif
			}
			else {
					_my_instance->WaitingCopybackWrites->push_back(dieBKE);
			} 
			break;
		case CMD_PROGRAM_PAGE:
		case CMD_PROGRAM_PAGE_MULTIPLANE:
		case CMD_PROGRAM_PAGE_COPYBACK:
		case CMD_PROGRAM_PAGE_COPYBACK_MULTIPLANE:
		{
			//说明闪存芯片上的写操作已经完成
			DEBUG("Chip " << chip->ChannelID << ", " << chip->ChipID << ": finished program command")
				int i = 0;
			for (std::list<NVM_Transaction_Flash*>::iterator it = dieBKE->ActiveTransactions.begin();
				it != dieBKE->ActiveTransactions.end(); it++, i++)
			{
				((NVM_Transaction_Flash_WR*)(*it))->Content = command->Meta_data[i].LPA;
				_my_instance->broadcastTransactionServicedSignal(*it);
			}
			dieBKE->ActiveTransactions.clear();										//因为闪存芯片上的写操作已经完成，此时删除掉对应dieBKE上的ActiveTransactions即可
			dieBKE->ClearCommand();													//删除dieBKE中的这个命令

			chipBKE->No_of_active_dies--;
			if (chipBKE->No_of_active_dies == 0 && chipBKE->WaitingReadTXCount == 0)
				chipBKE->Status = ChipStatus::IDLE;
			//Since the time required to send the resume command is very small, we ignore it
			if (chipBKE->Status == ChipStatus::IDLE)
				if (chipBKE->HasSuspend)
					_my_instance->send_resume_command_to_chip(chip, chipBKE);
			break;
		}
		case CMD_ERASE_BLOCK:
		case CMD_ERASE_BLOCK_MULTIPLANE:
			DEBUG("Chip " << chip->ChannelID << ", " << chip->ChipID << ": finished erase command")
				for (std::list<NVM_Transaction_Flash*>::iterator it = dieBKE->ActiveTransactions.begin();
					it != dieBKE->ActiveTransactions.end(); it++)
					_my_instance->broadcastTransactionServicedSignal(*it);
			dieBKE->ActiveTransactions.clear();
			dieBKE->ClearCommand();

			chipBKE->No_of_active_dies--;
			if (chipBKE->No_of_active_dies == 0 && chipBKE->WaitingReadTXCount == 0)
				chipBKE->Status = ChipStatus::IDLE;
			//Since the time required to send the resume command is very small, we ignore it
			if (chipBKE->Status == ChipStatus::IDLE)
				if (chipBKE->HasSuspend)
					_my_instance->send_resume_command_to_chip(chip, chipBKE);
			break;
		default:
			break;
		}

		//if (_my_instance->channels[chip->ChannelID]->GetStatus() == BusChannelStatus::IDLE)
		//	_my_instance->broadcastChannelIdleSignal(chip->ChannelID);
		//else if (chipBKE->Status == ChipStatus::IDLE)
		//	_my_instance->broadcastChipIdleSignal(chip);
		if (chipBKE->Status == ChipStatus::IDLE)
			_my_instance->broadcastChipIdleSignal(chip);
	}


	//读操作将数据从芯片传到控制器的时间在这个函数中计算，并注册相应的传输事件
	inline void NVM_PHY_ONFI_NVDDR2::transfer_read_data_from_chip(ChipBookKeepingEntry* chipBKE, DieBookKeepingEntry* dieBKE, NVM_Transaction_Flash* tr,std::list<NVM_Transaction_Flash*> Transactions)
	{
		//DEBUG2("Chip " << tr->Address.ChannelID << ", " << tr->Address.ChipID << ": transfer read data started for LPA: " << tr->LPA)
		dieBKE->ActiveTransfer = tr;
		////////////////////////////////////////////////////////////////////////////////////////////
		flash_channel_ID_type channel_id = dieBKE->ActiveTransactions.front()->Address.ChannelID;		//再通过dieBKE获取channel的id
		ONFI_Channel_NVDDR2* targetChannel = channels[channel_id];
		NVM::FlashMemory::Flash_Chip* targetChip = targetChannel->Chips[dieBKE->ActiveTransactions.front()->Address.ChipID];  //获取事务的目标chip

		//if (channel_id == 4 && targetChip->ChipID == 1 && Simulator->Time() > 30000) {
		//	std::cout << "ActiveTransfer  at " << Simulator->Time() << std::endl;
		//}
		////////////////////////////////////////////////////////////////////////////////////////////

		channels[tr->Address.ChannelID]->Chips[tr->Address.ChipID]->StartDataOutXfer();
		chipBKE->Status = ChipStatus::DATA_OUT;

		Simulator->Register_sim_event(Simulator->Time() + NVDDR2DataOutTransferTime(tr->Data_and_metadata_size_in_byte, channels[tr->Address.ChannelID]),
			this, dieBKE, (int)NVDDR2_SimEventType::READ_DATA_TRANSFERRED);

		tr->STAT_transfer_time += NVDDR2DataOutTransferTime(tr->Data_and_metadata_size_in_byte, channels[tr->Address.ChannelID]);

		//channels[tr->Address.ChannelID]->SetStatus(BusChannelStatus::BUSY, channels[tr->Address.ChannelID]->Chips[tr->Address.ChipID]);
	}

	void NVM_PHY_ONFI_NVDDR2::perform_interleaved_cmd_data_transfer(NVM::FlashMemory::Flash_Chip* chip, DieBookKeepingEntry* bookKeepingEntry)
	{
		ONFI_Channel_NVDDR2* target_channel = channels[bookKeepingEntry->ActiveTransactions.front()->Address.ChannelID];
		/*if (target_channel->Status == BusChannelStatus::BUSY)
			PRINT_ERROR("Requesting communication on a busy bus!")*/

		switch (bookKeepingEntry->ActiveTransactions.front()->Type)
		{
		case Transaction_Type::READ:
			chip->StartCMDXfer();
			bookKeepingTable[chip->ChannelID][chip->ChipID].Status = ChipStatus::CMD_IN;
			Simulator->Register_sim_event(Simulator->Time() + bookKeepingEntry->DieInterleavedTime,
				this, bookKeepingEntry, (int)NVDDR2_SimEventType::READ_CMD_ADDR_TRANSFERRED);       //注册新的读命令事件
			break;
		case Transaction_Type::WRITE:
			if (((NVM_Transaction_Flash_WR*)bookKeepingEntry->ActiveTransactions.front())->RelatedRead == NULL) {
				chip->StartCMDDataInXfer();
				bookKeepingTable[chip->ChannelID][chip->ChipID].Status = ChipStatus::CMD_DATA_IN;
				Simulator->Register_sim_event(Simulator->Time() + bookKeepingEntry->DieInterleavedTime,
					this, bookKeepingEntry, (int)NVDDR2_SimEventType::PROGRAM_CMD_ADDR_DATA_TRANSFERRED);
			}
			else {
				chip->StartCMDXfer();
				bookKeepingTable[chip->ChannelID][chip->ChipID].Status = ChipStatus::CMD_IN;
				Simulator->Register_sim_event(Simulator->Time() + bookKeepingEntry->DieInterleavedTime, this,
					bookKeepingEntry, (int)NVDDR2_SimEventType::READ_CMD_ADDR_TRANSFERRED);
			}
			break;
		case Transaction_Type::ERASE:
			chip->StartCMDXfer();
			bookKeepingTable[chip->ChannelID][chip->ChipID].Status = ChipStatus::CMD_IN;
			Simulator->Register_sim_event(Simulator->Time() + bookKeepingEntry->DieInterleavedTime,
				this, bookKeepingEntry, (int)NVDDR2_SimEventType::ERASE_SETUP_COMPLETED);
			break;
		default:
			PRINT_ERROR("NVMController_NVDDR2: Uknown flash transaction type!")
		}
		target_channel->SetStatus(BusChannelStatus::BUSY, chip);
	}


	//会将整个chip上所有die上挂起的事务都恢复
	inline void NVM_PHY_ONFI_NVDDR2::send_resume_command_to_chip(NVM::FlashMemory::Flash_Chip* chip, ChipBookKeepingEntry* chipBKE)
	{
		//DEBUG2("Chip " << chip->ChannelID << ", " << chip->ChipID << ": resume command " )
		for (unsigned int i = 0; i < die_no_per_chip; i++) {				//循环，把chip上的每一个die挂起的事务都恢复
			DieBookKeepingEntry* dieBKE = &chipBKE->Die_book_keeping_records[i];
			//Since the time required to send the resume command is very small, MQSim ignores it to simplify the simulation
			dieBKE->PrepareResume();										//将这个chip上每个die上的挂起事务加入到各自die的ActiveTransactions中
			chipBKE->PrepareResume();
			chip->Resume(dieBKE->ActiveCommand->Address[0].DieID);			//经过了上面的dieBKE->PrepareResume之后，现在的ActiveCommand就是挂起的命令
			switch (dieBKE->ActiveCommand->CommandCode) {
			case CMD_READ_PAGE:
			case CMD_READ_PAGE_MULTIPLANE:
			case CMD_READ_PAGE_COPYBACK:
			case CMD_READ_PAGE_COPYBACK_MULTIPLANE:
				chipBKE->Status = ChipStatus::READING;
				break;
			case CMD_PROGRAM_PAGE:
			case CMD_PROGRAM_PAGE_MULTIPLANE:
			case CMD_PROGRAM_PAGE_COPYBACK:
			case CMD_PROGRAM_PAGE_COPYBACK_MULTIPLANE:
				chipBKE->Status = ChipStatus::WRITING;
				break;
			case CMD_ERASE_BLOCK:
			case CMD_ERASE_BLOCK_MULTIPLANE:
				chipBKE->Status = ChipStatus::ERASING;
				break;
			}

		}
	}
}
