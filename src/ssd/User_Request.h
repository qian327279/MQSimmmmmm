#ifndef USER_REQUEST_H
#define USER_REQUEST_H

#include <string>
#include <list>
#include "SSD_Defs.h"
#include "../sim/Sim_Defs.h"
#include "Host_Interface_Defs.h"
#include "NVM_Transaction.h"

namespace SSD_Components
{
	enum class UserRequestType { READ, WRITE };
	class NVM_Transaction;
	class User_Request
	{
	public:
		User_Request();
		IO_Flow_Priority_Class::Priority Priority_class;
		io_request_id_type ID;
		LHA_type Start_LBA;

		sim_time_type STAT_InitiationTime;     //请求的启动时间
		sim_time_type STAT_ResponseTime;	   //请求的响应时间
		std::list<NVM_Transaction*> Transaction_list;			//包含与该用户请求关联的所有NVM事务的列表
		unsigned int Sectors_serviced_from_cache;

		unsigned int Size_in_byte;
		unsigned int SizeInSectors;
		UserRequestType Type;
		stream_id_type Stream_id;				//所属的流ID
		bool ToBeIgnored;
		void* IO_command_info;//used to store host I/O command info
		void* Data;
	private:
		static unsigned int lastId;				
	};
}

#endif // !USER_REQUEST_H
