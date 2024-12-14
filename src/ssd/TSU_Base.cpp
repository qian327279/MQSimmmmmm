#include <string>
#include "TSU_Base.h"

#define TRTOSTR(TR) (TR->Type == Transaction_Type::READ ? "Read, " : (TR->Type == Transaction_Type::WRITE ? "Write, " : "Erase, ") )

namespace SSD_Components
{
	TSU_Base* TSU_Base::_my_instance = NULL;

	TSU_Base::TSU_Base(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI_NVDDR2* NVMController, Flash_Scheduling_Type Type,
		unsigned int ChannelCount, unsigned int chip_no_per_channel, unsigned int DieNoPerChip, unsigned int PlaneNoPerDie,
		bool EraseSuspensionEnabled, bool ProgramSuspensionEnabled,
		sim_time_type WriteReasonableSuspensionTimeForRead,
		sim_time_type EraseReasonableSuspensionTimeForRead,
		sim_time_type EraseReasonableSuspensionTimeForWrite)
		: Sim_Object(id), ftl(ftl), _NVMController(NVMController), type(Type),
		channel_count(ChannelCount), chip_no_per_channel(chip_no_per_channel), die_no_per_chip(DieNoPerChip), plane_no_per_die(PlaneNoPerDie),
		eraseSuspensionEnabled(EraseSuspensionEnabled), programSuspensionEnabled(ProgramSuspensionEnabled),
		writeReasonableSuspensionTimeForRead(WriteReasonableSuspensionTimeForRead), eraseReasonableSuspensionTimeForRead(EraseReasonableSuspensionTimeForRead),
		eraseReasonableSuspensionTimeForWrite(EraseReasonableSuspensionTimeForWrite), opened_scheduling_reqs(0)
	{
		_my_instance = this;
		Round_robin_turn_of_channel = new flash_chip_ID_type[channel_count];			//为每个通道存储一个值，这个值用于记录当前通道轮流到哪个芯片了。
		for (unsigned int channelID = 0; channelID < channel_count; channelID++) {
			Round_robin_turn_of_channel[channelID] = 0;									//每个通道初始化为0，表示每个通道目前轮到的是第一个chip。
		}
	}

	TSU_Base::~TSU_Base()
	{
		delete[] Round_robin_turn_of_channel;
	}

	void TSU_Base::Setup_triggers()
	{
		Sim_Object::Setup_triggers();
		_NVMController->ConnectToTransactionServicedSignal(handle_transaction_serviced_signal_from_PHY);//当（PHY）处理器（如 _NVMController）接收到并服务完一个事务后调用handle_transaction_serviced_signal_from_PHY函数
		_NVMController->ConnectToChannelIdleSignal(handle_channel_idle_signal);
		_NVMController->ConnectToChipIdleSignal(handle_chip_idle_signal);
	}

	void TSU_Base::handle_transaction_serviced_signal_from_PHY(NVM_Transaction_Flash* transaction)
	{
		//TSU does nothing. The generator of the transaction will handle it.
	}

	void TSU_Base::handle_channel_idle_signal(flash_channel_ID_type channelID)//通道空闲时触发，这个函数时重新安排或调度下一个待处理的任务，以充分利用系统资源
	{
		if (_my_instance->_NVMController->check_idle_controller_exist()) {
			for (unsigned int i = 0; i < _my_instance->chip_no_per_channel; i++) {

				if (!_my_instance->_NVMController->check_idle_controller_exist()) {
					break;
				}
				_my_instance->process_chip_requests(_my_instance->_NVMController->Get_chip(channelID, _my_instance->Round_robin_turn_of_channel[channelID]));
				_my_instance->Round_robin_turn_of_channel[channelID] = (flash_chip_ID_type)(_my_instance->Round_robin_turn_of_channel[channelID] + 1) % _my_instance->chip_no_per_channel;

				//A transaction has been started, so TSU should stop searching for another chip
			
				//if (_my_instance->_NVMController->Get_channel_status(channelID) == BusChannelStatus::BUSY) {
				//	break;
				//}
			}
		}
	}

	void TSU_Base::handle_chip_idle_signal(NVM::FlashMemory::Flash_Chip* chip)
	{
		//if (_my_instance->_NVMController->Get_channel_status(chip->ChannelID) == BusChannelStatus::IDLE) {//检查对应chip所在的channel是否空闲
		//	_my_instance->process_chip_requests(chip);
		//}
		if (_my_instance->_NVMController->check_idle_controller_exist()) {		//存在可用控制器再进入
			_my_instance->process_chip_requests(chip);
		}
	}

	void TSU_Base::Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter)
	{
	}


	//从sourceQueue1中选取事务到transaction_dispatch_slots中，transaction_dispatch_slots存储的事务目标chip相同，且die也相同，真正的一些发送命令相关的时延及状态记录的改变在Send_command_to_chip函数中。
	bool TSU_Base::issue_command_to_chip(Flash_Transaction_Queue* sourceQueue1, Flash_Transaction_Queue* sourceQueue2, Transaction_Type transactionType, bool suspensionRequired, flash_channel_ID_type ChannelID, flash_chip_ID_type ChipID)
	{
		flash_die_ID_type dieID = sourceQueue1->front()->Address.DieID;			//从sourceQueue1获取第一个事务的dieID和pageID
		flash_page_ID_type pageID = sourceQueue1->front()->Address.PageID;
		unsigned int planeVector = 0;											
		static int issueCntr = 0;                                               
		int flag = 0;															//用于判断请求是从sourceQueue1获得的还是从sourceQueue2获得的
		for (unsigned int i = 0; i < die_no_per_chip; i++)						//遍历所有的die
		{
			transaction_dispatch_slots.clear();
			planeVector = 0;													
			//把dieID相同，PlaneID相同，pageID相同的请求都放到了transaction_dispatch_slots中
			for (Flash_Transaction_Queue::iterator it = sourceQueue1->begin(); it != sourceQueue1->end();)
			{
				if (transaction_is_ready(*it) && (*it)->Address.DieID == dieID && !(planeVector & 1 << (*it)->Address.PlaneID)) //planeVector & 1 << (*it)->Address.PlaneID)用于判断planeVector中是否已经标记了(*it)->Address.PlaneID 所指示的平面
				{
					//Check for identical pages when running multiplane command
					if (planeVector == 0 || (*it)->Address.PageID == pageID)
					{
						(*it)->SuspendRequired = suspensionRequired;
						planeVector |= 1 << (*it)->Address.PlaneID;				//将这个PlaneID标记
						transaction_dispatch_slots.push_back(*it);				//将事务添加到transaction_dispatch_slots
						flag = 1;
						DEBUG(issueCntr++ << ": " << Simulator->Time() << " Issueing Transaction - Type:" << TRTOSTR((*it)) << ", PPA:" << (*it)->PPA << ", LPA:" << (*it)->LPA << ", Channel: " << (*it)->Address.ChannelID << ", Chip: " << (*it)->Address.ChipID);
						/*sourceQueue1->remove(it++);*/								//将事务从从sourceQueue1中移除
						continue;
					}
				}
				it++;
			}

			if (sourceQueue2 != NULL && transaction_dispatch_slots.size() < plane_no_per_die)			//如果transaction_dispatch_slots事务数量小于每个die的Plane数量，将尝试从sourceQueue2中选择事务
			{
				for (Flash_Transaction_Queue::iterator it = sourceQueue2->begin(); it != sourceQueue2->end();)
				{
					if (transaction_is_ready(*it) && (*it)->Address.DieID == dieID && !(planeVector & 1 << (*it)->Address.PlaneID))
					{
						//Check for identical pages when running multiplane command
						if (planeVector == 0 || (*it)->Address.PageID == pageID)
						{
							(*it)->SuspendRequired = suspensionRequired;
							planeVector |= 1 << (*it)->Address.PlaneID;
							transaction_dispatch_slots.push_back(*it);
							flag = 2;
							DEBUG(issueCntr++ << ": " << Simulator->Time() << " Issueing Transaction - Type:" << TRTOSTR((*it)) << ", PPA:" << (*it)->PPA << ", LPA:" << (*it)->LPA << ", Channel: " << (*it)->Address.ChannelID << ", Chip: " << (*it)->Address.ChipID);
							/*sourceQueue2->remove(it++);*/
							continue;
						}
					}
					it++;
				}
			}

			if (transaction_dispatch_slots.size() > 0)
			{
				ChipBookKeepingEntry** BookKeepingEntry = _my_instance->_NVMController->get_ChipBookKeepingEntry();
				ChipBookKeepingEntry* chipBKE = &BookKeepingEntry[transaction_dispatch_slots.front()->Address.ChannelID][transaction_dispatch_slots.front()->Address.ChipID];
				DieBookKeepingEntry* dieBKE = &chipBKE->Die_book_keeping_records[transaction_dispatch_slots.front()->Address.DieID];

				if (dieBKE->Free==false) {									//这段代码原本是send_command_to_chip中用于判断违规挂起操作的，现在被移动到这个位置，防止两个请求同时预定同一个chip上的同一个die
					transaction_dispatch_slots.clear();
					dieID = (dieID + 1) % die_no_per_chip;
					return false;
				}

				std::vector<int> reserve_path;
				int controller_num;
				if (_my_instance->_NVMController->Route_reservation(ChannelID, ChipID, reserve_path, controller_num)) {					//假设路径预定成功
					dieBKE->Free = false;
					_my_instance->_NVMController->Controller_state[controller_num] = 1;
					dieBKE->ActiveCommand = new NVM::FlashMemory::Flash_Command();

					for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_dispatch_slots.begin();   //将command的一些信息放进了dieBKE中
						it != transaction_dispatch_slots.end(); it++) {
						dieBKE->ActiveTransactions.push_back(*it);								  //被移动到issue_command_to_chip中
						dieBKE->ActiveCommand->Address.push_back((*it)->Address);
						NVM::FlashMemory::PageMetadata metadata;
						metadata.LPA = (*it)->LPA;
						dieBKE->ActiveCommand->Meta_data.push_back(metadata);
					}

					dieBKE->temporary_transactions = transaction_dispatch_slots;				  //将有效信息存储进dieBKE中
					dieBKE->ActiveCommand->controller_number = controller_num;
					dieBKE->ActiveCommand->path = reserve_path;

					Simulator->Register_sim_event(Simulator->Time() + 120* reserve_path.size(), _NVMController, dieBKE, (int)NVDDR2_SimEventType::ROUTE_RESERVATION);//将tr+2(ts+tw)设置为20个时钟周期

					if (flag == 1) {																								//只有路径预定成功了才能移除事务，原代码在预定失败时也会移除事务
						for (auto it = transaction_dispatch_slots.begin(); it != transaction_dispatch_slots.end(); ++it) {
							auto transaction = *it;
							auto found = std::find(sourceQueue1->begin(), sourceQueue1->end(), transaction);
							if (found != sourceQueue1->end()) {
								sourceQueue1->remove(transaction);
							}
						}
					}
					
					else if (flag == 2) {
						for (auto it = transaction_dispatch_slots.begin(); it != transaction_dispatch_slots.end(); ++it) {
							auto transaction = *it;
							auto found = std::find(sourceQueue2->begin(), sourceQueue2->end(), transaction);
							if (found != sourceQueue2->end()) {
								sourceQueue2->remove(transaction);
							}
						}
					}
				
					transaction_dispatch_slots.clear();
					dieID = (dieID + 1) % die_no_per_chip;																//更新dieID，用于寻找下一轮可以放入transaction_dispatch_slots的请求
					return true;

				}
				else {
					transaction_dispatch_slots.clear();
					dieID = (dieID + 1) % die_no_per_chip;
					return false;
				}

			}
			else
			{
				transaction_dispatch_slots.clear();
				dieID = (dieID + 1) % die_no_per_chip;
				return false;
			}
		}

		return false;
	}
}
