#ifndef NVM_PHY_ONFI_NVDDR2_H
#define NVM_PHY_ONFI_NVDDR2_H

#include <queue>
#include <list>
#include "../sim/Sim_Defs.h"
#include "../nvm_chip/flash_memory/FlashTypes.h"
#include "../nvm_chip/flash_memory/Flash_Command.h"
#include "NVM_PHY_ONFI.h"
#include "ONFI_Channel_NVDDR2.h"
#include "Flash_Transaction_Queue.h"

namespace SSD_Components
{
	enum class NVDDR2_SimEventType
	{
		READ_DATA_TRANSFERRED, READ_CMD_ADDR_TRANSFERRED,
		PROGRAM_CMD_ADDR_DATA_TRANSFERRED,
		PROGRAM_COPYBACK_CMD_ADDR_TRANSFERRED,
		ERASE_SETUP_COMPLETED,
		ROUTE_RESERVATION
	};

	enum class PHY_Status {
		IDLE, BUSY
	};

	enum class PortDirection {
		Port_UP = 0,
		Port_RIGHT = 1,
		Port_DOWN = 2,
		Port_LEFT = 3,

	};

	class DieBookKeepingEntry
	{
	public:
		NVM::FlashMemory::Flash_Command* ActiveCommand; //The current command that is executing on the die

		/*The current transactions that are being serviced. For the set of transactions in ActiveTransactions,
		there is one ActiveCommand that is geting executed on the die. Transaction is a FTL-level concept, and
		command is a flash chip-level concept*/
		std::list<NVM_Transaction_Flash*> ActiveTransactions;					//指向当前在die上执行的命令指针
		std::list<NVM_Transaction_Flash*> temporary_transactions;				//用于暂存留给send_to_command的事务
		NVM::FlashMemory::Flash_Command* SuspendedCommand;
		std::list<NVM_Transaction_Flash*> SuspendedTransactions;
		NVM_Transaction_Flash* ActiveTransfer; //The current transaction 
		bool Free;
		bool Suspended;										
		sim_time_type Expected_finish_time;
		sim_time_type RemainingExecTime;
		sim_time_type DieInterleavedTime;//If the command transfer is done in die-interleaved mode, the transfer time is recorded in this temporary variable

		void PrepareSuspend()
		{
			SuspendedCommand = ActiveCommand;
			RemainingExecTime = Expected_finish_time - Simulator->Time();
			SuspendedTransactions.insert(SuspendedTransactions.begin(), ActiveTransactions.begin(), ActiveTransactions.end());
			Suspended = true;
			ActiveCommand = NULL;
			ActiveTransactions.clear();
			Free = true;
		}

		void PrepareResume()
		{
			ActiveCommand = SuspendedCommand;
			Expected_finish_time = Simulator->Time() + RemainingExecTime;
			ActiveTransactions.insert(ActiveTransactions.begin(), SuspendedTransactions.begin(), SuspendedTransactions.end());  //将挂起的事务插入到活动事务列表中
			Suspended = false;
			SuspendedCommand = NULL;
			SuspendedTransactions.clear();
			Free = false;
		}

		void ClearCommand()
		{
			delete ActiveCommand;
			ActiveCommand = NULL;
			ActiveTransactions.clear();
			Free = true;
		}
	};

	class ChipBookKeepingEntry
	{
	public:
		ChipStatus Status;
		DieBookKeepingEntry* Die_book_keeping_records;
		sim_time_type Expected_command_exec_finish_time;
		sim_time_type Last_transfer_finish_time;
		bool HasSuspend;
		std::queue<DieBookKeepingEntry*> OngoingDieCMDTransfers;    //正在进行命令传输的die队列
		unsigned int WaitingReadTXCount;							//当前等待的读事务数量
		unsigned int No_of_active_dies;								//当前正忙的die的数量

		void PrepareSuspend() { HasSuspend = true; No_of_active_dies = 0; }
		void PrepareResume() { HasSuspend = false; }
	};

	class NVM_PHY_ONFI_NVDDR2 : public NVM_PHY_ONFI
	{
	public:
		NVM_PHY_ONFI_NVDDR2(const sim_object_id_type& id, ONFI_Channel_NVDDR2** channels,
			unsigned int ChannelCount, unsigned int chip_no_per_channel, unsigned int DieNoPerChip, unsigned int PlaneNoPerDie);
		void Setup_triggers();
		void Validate_simulation_config();
		void Start_simulation();
		bool check_idle_controller_exist();
		void Send_command_to_chip(std::list<NVM_Transaction_Flash*>& transactionList);
		void Change_flash_page_status_for_preconditioning(const NVM::FlashMemory::Physical_Page_Address& page_address, const LPA_type lpa);
		void Execute_simulator_event(MQSimEngine::Sim_Event*);
		BusChannelStatus Get_channel_status(flash_channel_ID_type channelID);
		bool Route_reservation(flash_channel_ID_type channelID, flash_chip_ID_type chipID, std::vector<int>& path, int& controller_no);
		NVM::FlashMemory::Flash_Chip* Get_chip(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id);
		LPA_type Get_metadata(flash_channel_ID_type channe_id, flash_chip_ID_type chip_id, flash_die_ID_type die_id, flash_plane_ID_type plane_id, flash_block_ID_type block_id, flash_page_ID_type page_id);//A simplification to decrease the complexity of GC execution! The GC unit may need to know the metadata of a page to decide if a page is valid or invalid. 
		bool HasSuspendedCommand(NVM::FlashMemory::Flash_Chip* chip);
		ChipStatus GetChipStatus(NVM::FlashMemory::Flash_Chip* chip);
		sim_time_type Expected_finish_time(NVM::FlashMemory::Flash_Chip* chip);
		sim_time_type Expected_finish_time(NVM_Transaction_Flash* transaction);
		sim_time_type Expected_transfer_time(NVM_Transaction_Flash* transaction);
		NVM_Transaction_Flash* Is_chip_busy_with_stream(NVM_Transaction_Flash* transaction);
		bool Is_chip_busy(NVM_Transaction_Flash* transaction);
		void Change_memory_status_preconditioning(const NVM::NVM_Memory_Address* address, const void* status_info);
		ChipBookKeepingEntry** get_ChipBookKeepingEntry();
		int* Controller_state;								//用一个数组记录控制器的空闲情况，0表示目前状态为空闲，1表示正在忙碌
	private:
		void transfer_read_data_from_chip(ChipBookKeepingEntry* chipBKE, DieBookKeepingEntry* dieBKE, NVM_Transaction_Flash* tr, std::list<NVM_Transaction_Flash*> Transactions);
		void perform_interleaved_cmd_data_transfer(NVM::FlashMemory::Flash_Chip* chip, DieBookKeepingEntry* bookKeepingEntry);
		void send_resume_command_to_chip(NVM::FlashMemory::Flash_Chip* chip, ChipBookKeepingEntry* chipBKE);
		static void handle_ready_signal_from_chip(NVM::FlashMemory::Flash_Chip* chip, NVM::FlashMemory::Flash_Command* command);
		int channel_num;									//记录通道的数量
		static NVM_PHY_ONFI_NVDDR2* _my_instance;
		ONFI_Channel_NVDDR2** channels;
		ChipBookKeepingEntry** bookKeepingTable;
		Flash_Transaction_Queue* WaitingReadTX, * WaitingGCRead_TX, * WaitingMappingRead_TX;
		std::list<DieBookKeepingEntry*>* WaitingCopybackWrites;
		int*** Link_state;									//用三位数组Link_state记录当前2D-Mesh拓扑的链路状态，第一个维度是行，第二个维度是列，第三个维度只有0和1，分别用于表述是横向还是纵向的链路,值为-1表示是无效链路
		int Find_the_most_suitable_controller(flash_channel_ID_type channelID);//根据当前的控制器状态及目标芯片，找到离得最近的控制器，找不到空闲的控制器时，函数会返回-1这个值
		bool Determines_whether_the_wait_queue_can_be_executed(const std::vector<int>& vec, int*** Link_state, int controller_num);	//判断waiting队列内的事务能不能执行
		void Set_link_status_based_on_path(const std::vector<int>& vec, int*** Link_state, int controller_num);						//当路径预定成功后，根据路径，将对应的链路状态都设置为繁忙
		void Set_link_status_to_idle_based_on_the_path(const std::vector<int>& vec, int*** Link_state, int controller_num);			//当请求完成之后，根据路径，将对应链路状态都设置为空闲
		void get_the_best_port_attempt_order(PortDirection* attempt_sequence, flash_channel_ID_type target_channelID, flash_chip_ID_type target_chip_id, flash_channel_ID_type current_channelID, flash_chip_ID_type current_chip_id);
		void Port_index_converted_to_link_index(int port_channelID, int port_chipID, int port_direction, int* check_array);
		void update_coordinate(int& current_channelID, int& current_chipID, int port_direction);
		std::pair<int, int> get_updated_coordinate(int current_channelID, int current_chipID, int port_direction);
		bool whether_a_ring_is_formed(std::vector<std::pair<int, int>> Passed_coordinates, std::pair<int, int>coordinate);
		bool Backtracking_or_not(int*** Route_port_record, int current_channelID, int current_chipID);		//决定是否回溯到上一个节点
		int port_inversion(int port_num);																	//将端口方向完全反转，如上端口转成下端口，左端口转成右端口						
	};
}

#endif // !NVM_PHY_ONFI_NVDDR2_H
