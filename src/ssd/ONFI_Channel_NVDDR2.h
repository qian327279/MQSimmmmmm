#ifndef ONFI_CHANNEL_NVDDR2_H
#define ONFI_CHANNEL_NVDDR2_H

#include "../nvm_chip/flash_memory/FlashTypes.h"
#include "ONFI_Channel_Base.h"


#define NVDDR2DataInTransferTime(X,Y) ((X / Y->ChannelWidth / 2) * Y->TwoUnitDataInTime)   //在这个式子中，TwoUnitDataInTime表示的是channel传输一次数据所需要的纳秒数，而ChannelWidth代表的是传输一次的byte数，用x除以ChannelWidth代表这次传输需要传输的次数，乘以TwoUnitDataInTime代表传输这么多次数据总共需要多少纳秒
#define NVDDR2DataOutTransferTime(X,Y) ((X / Y->ChannelWidth / 2) * Y->TwoUnitDataOutTime)


namespace SSD_Components
{
	class ONFI_Channel_NVDDR2 : public ONFI_Channel_Base
	{
	public:
		ONFI_Channel_NVDDR2(flash_channel_ID_type channelID, unsigned int chipCount, NVM::FlashMemory::Flash_Chip** flashChips, unsigned int ChannelWidth,
			sim_time_type t_RC = 6, sim_time_type t_DSC = 6,
			sim_time_type t_DBSY = 500, sim_time_type t_CS = 20, sim_time_type t_RR = 20,
			sim_time_type t_WB = 100, sim_time_type t_WC = 25, sim_time_type t_ADL = 70, sim_time_type t_CALS = 15,
			sim_time_type t_DQSRE = 15, sim_time_type t_RPRE = 15, sim_time_type t_RHW = 100, sim_time_type t_CCS = 300,
			sim_time_type t_WPST = 6, sim_time_type t_WPSTH = 15);
		//其中t_RC=(double)1000 / parameters->Channel_Transfer_Rate) * 2，因为最后的单位是纳秒，而parameters->Channel_Transfer_Rate自带10的6次方，所以乘以10的3次方，计算出的结果就是一次传输需要的时间，单位为纳秒

		sim_time_type TwoUnitDataOutTime; //The DDR delay for two-unit device data out
		sim_time_type ReadCommandTime[5];//Read command transfer time for different number of planes
		sim_time_type ReadDataOutSetupTime, ReadDataOutSetupTime_TwoPlane, ReadDataOutSetupTime_ThreePlane, ReadDataOutSetupTime_FourPlane;

		sim_time_type TwoUnitDataInTime; //The DDR delay for two-unit device data in
		sim_time_type ProgramCommandTime[5];//Program command transfer time for different number of planes
		sim_time_type ProgramSuspendCommandTime;

		sim_time_type EraseCommandTime[5];
		sim_time_type EraseSuspendCommandTime;

		unsigned int ChannelWidth; //channel width in bytes
	private:
		//Data input/ouput timing parameters related to bus frequency
		sim_time_type t_RC; //Average RE cycle time, e.g. 6ns
		sim_time_type t_DSC; //Average DQS cycle time, e.g. 6ns

		//flash timing parameters
		sim_time_type t_DBSY; //Dummy busy time, e.g. 500ns
	
		//ONFI NVDDR2 command and address protocol timing parameters
		sim_time_type t_CS; //CE setup, e.g. 20ns
		sim_time_type t_RR; //Ready to data output, e.g. 20ns
		sim_time_type t_WB; //CLK HIGH to R/B LOW, e.g. 100ns
		sim_time_type t_WC; //WE cycle time, e.g. 25ns
		sim_time_type t_ADL; //ALE to data loading time, e.g. 70ns
		sim_time_type t_CALS; //ALE, CLE setup with ODT disabled, e.g. 15
		
		//ONFI NVDDR2 data output protocol timing paramters
		sim_time_type t_DQSRE; //Access window of DQS from RE, e.g. 15ns
		sim_time_type t_RPRE; //Read preamble, e.g. 15ns
		sim_time_type t_RHW; //Data output to command, address, or data input, 100ns
		sim_time_type t_CCS; //Change column setup time to data in/out or next command, 300ns
		
		//ONFI NVDDR2 data input protocol timing paramters
		sim_time_type t_WPST; //DQS write postamble, e.g. 6ns
		sim_time_type t_WPSTH; //DQS write postamble hold time, e.g. 15ns
	};
}

#endif // !ONFI_CHANNEL_NVDDR2_H
