#include "TSU_Priority_OutOfOrder.h"
#include "Host_Interface_Defs.h"

namespace SSD_Components
{

TSU_Priority_OutOfOrder::TSU_Priority_OutOfOrder(const sim_object_id_type &id,
                                                 FTL *ftl,
                                                 NVM_PHY_ONFI_NVDDR2 *NVMController,
                                                 unsigned int ChannelCount,
                                                 unsigned int ChipNoPerChannel,
                                                 unsigned int DieNoPerChip,
                                                 unsigned int PlaneNoPerDie,
                                                 sim_time_type WriteReasonableSuspensionTimeForRead,
                                                 sim_time_type EraseReasonableSuspensionTimeForRead,
                                                 sim_time_type EraseReasonableSuspensionTimeForWrite,
                                                 bool EraseSuspensionEnabled,
                                                 bool ProgramSuspensionEnabled)
    : TSU_Base(id,
               ftl,
               NVMController,
               Flash_Scheduling_Type::PRIORITY_OUT_OF_ORDER,
               ChannelCount,
               ChipNoPerChannel,
               DieNoPerChip,
               PlaneNoPerDie,
               WriteReasonableSuspensionTimeForRead,
               EraseReasonableSuspensionTimeForRead,
               EraseReasonableSuspensionTimeForWrite,
               EraseSuspensionEnabled,
               ProgramSuspensionEnabled)
{
    UserReadTRQueue = new Flash_Transaction_Queue **[channel_count];
    UserWriteTRQueue = new Flash_Transaction_Queue **[channel_count];
    GCReadTRQueue = new Flash_Transaction_Queue *[channel_count];
    GCWriteTRQueue = new Flash_Transaction_Queue *[channel_count];
    GCEraseTRQueue = new Flash_Transaction_Queue *[channel_count];
    MappingReadTRQueue = new Flash_Transaction_Queue *[channel_count];
    MappingWriteTRQueue = new Flash_Transaction_Queue *[channel_count];
    nextPriorityClassRead = new IO_Flow_Priority_Class::Priority *[channel_count];
    nextPriorityClassWrite = new IO_Flow_Priority_Class::Priority * [channel_count];
    currentWeightRead = new int* [channel_count];
    currentWeightWrite = new int *[channel_count];
    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {//将队列初始化好
        UserReadTRQueue[channelID] = new Flash_Transaction_Queue *[chip_no_per_channel];
        UserWriteTRQueue[channelID] = new Flash_Transaction_Queue *[chip_no_per_channel];
        GCReadTRQueue[channelID] = new Flash_Transaction_Queue[chip_no_per_channel];
        GCWriteTRQueue[channelID] = new Flash_Transaction_Queue[chip_no_per_channel];
        GCEraseTRQueue[channelID] = new Flash_Transaction_Queue[chip_no_per_channel];
        MappingReadTRQueue[channelID] = new Flash_Transaction_Queue[chip_no_per_channel];
        MappingWriteTRQueue[channelID] = new Flash_Transaction_Queue[chip_no_per_channel];
        nextPriorityClassRead[channelID] = new IO_Flow_Priority_Class::Priority[chip_no_per_channel];
        nextPriorityClassWrite[channelID] = new IO_Flow_Priority_Class::Priority[chip_no_per_channel];
        currentWeightRead[channelID] = new int[chip_no_per_channel];
        currentWeightWrite[channelID] = new int[chip_no_per_channel];
        for (unsigned int chipId = 0; chipId < chip_no_per_channel; chipId++)
        {
            UserReadTRQueue[channelID][chipId] = new Flash_Transaction_Queue[IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS];
            UserWriteTRQueue[channelID][chipId] = new Flash_Transaction_Queue[IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS];
            nextPriorityClassRead[channelID][chipId] = IO_Flow_Priority_Class::URGENT;      //每个芯片的下一个读优先级为ERGENT
            nextPriorityClassWrite[channelID][chipId] = IO_Flow_Priority_Class::URGENT;     //每个芯片的下一个写优先级为ERGENT
            currentWeightRead[channelID][chipId] = IO_Flow_Priority_Class::get_scheduling_weight(IO_Flow_Priority_Class::HIGH);
            currentWeightWrite[channelID][chipId] = IO_Flow_Priority_Class::get_scheduling_weight(IO_Flow_Priority_Class::HIGH);
            for (unsigned int priorityClass = 0; priorityClass < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priorityClass++)
            {//设置每个优先级类的事务队列ID，便于调试
                UserReadTRQueue[channelID][chipId][priorityClass].Set_id("User_Read_TR_Queue@" + std::to_string(channelID) + "@" + std::to_string(chipId) + "@" + IO_Flow_Priority_Class::to_string(priorityClass));    
                UserWriteTRQueue[channelID][chipId][priorityClass].Set_id("User_Write_TR_Queue@" + std::to_string(channelID) + "@" + std::to_string(chipId) + "@" + IO_Flow_Priority_Class::to_string(priorityClass));
            }

            GCReadTRQueue[channelID][chipId].Set_id("GC_Read_TR_Queue@" + std::to_string(channelID) + "@" + std::to_string(chipId));
            MappingReadTRQueue[channelID][chipId].Set_id("Mapping_Read_TR_Queue@" + std::to_string(channelID) + "@" + std::to_string(chipId));
            MappingWriteTRQueue[channelID][chipId].Set_id("Mapping_Write_TR_Queue@" + std::to_string(channelID) + "@" + std::to_string(chipId));
            GCWriteTRQueue[channelID][chipId].Set_id("GC_Write_TR_Queue@" + std::to_string(channelID) + "@" + std::to_string(chipId));
            GCEraseTRQueue[channelID][chipId].Set_id("GC_Erase_TR_Queue@" + std::to_string(channelID) + "@" + std::to_string(chipId));
        }
    }
}

TSU_Priority_OutOfOrder::~TSU_Priority_OutOfOrder()
{
    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {
        for (unsigned int chipId = 0; chipId < chip_no_per_channel; chipId++)
        {
            delete[] UserReadTRQueue[channelID][chipId];
            delete[] UserWriteTRQueue[channelID][chipId];
        }
        delete[] nextPriorityClassRead[channelID];
        delete[] nextPriorityClassWrite[channelID];
        delete[] currentWeightRead[channelID];
        delete[] currentWeightWrite[channelID];
        delete[] GCReadTRQueue[channelID];
        delete[] GCWriteTRQueue[channelID];
        delete[] GCEraseTRQueue[channelID];
        delete[] MappingReadTRQueue[channelID];
        delete[] MappingWriteTRQueue[channelID];
    }
    delete[] UserReadTRQueue;
    delete[] UserWriteTRQueue;
    delete[] nextPriorityClassRead;
    delete[] nextPriorityClassWrite;
    delete[] currentWeightRead;
    delete[] currentWeightWrite;
    delete[] GCReadTRQueue;
    delete[] GCWriteTRQueue;
    delete[] GCEraseTRQueue;
    delete[] MappingReadTRQueue;
    delete[] MappingWriteTRQueue;
}

void TSU_Priority_OutOfOrder::Start_simulation()
{
}

void TSU_Priority_OutOfOrder::Validate_simulation_config()
{
}

void TSU_Priority_OutOfOrder::Execute_simulator_event(MQSimEngine::Sim_Event *event)
{
}

void TSU_Priority_OutOfOrder::Report_results_in_XML(std::string name_prefix, Utils::XmlWriter &xmlwriter)
{
    name_prefix = name_prefix + ".TSU";
    xmlwriter.Write_open_tag(name_prefix);

    TSU_Base::Report_results_in_XML(name_prefix, xmlwriter);

    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {
        for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
        {
            for (unsigned int priorityClass = 0; priorityClass < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priorityClass++)
            {
                UserReadTRQueue[channelID][chip_cntr][priorityClass].Report_results_in_XML(name_prefix + ".User_Read_TR_Queue.Priority." + IO_Flow_Priority_Class::to_string(priorityClass), xmlwriter);
            }
        }
    }

    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {
        for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
        {
            for (unsigned int priorityClass = 0; priorityClass < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priorityClass++)
            {
                UserWriteTRQueue[channelID][chip_cntr][priorityClass].Report_results_in_XML(name_prefix + ".User_Write_TR_Queue.Priority." + IO_Flow_Priority_Class::to_string(priorityClass), xmlwriter);
            }
        }
    }

    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {
        for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
        {
            MappingReadTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".Mapping_Read_TR_Queue", xmlwriter);
        }
    }

    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {
        for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
        {
            MappingWriteTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".Mapping_Write_TR_Queue", xmlwriter);
        }
    }

    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {
        for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
        {
            GCReadTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Read_TR_Queue", xmlwriter);
        }
    }

    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {
        for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
        {
            GCWriteTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Write_TR_Queue", xmlwriter);
        }
    }

    for (unsigned int channelID = 0; channelID < channel_count; channelID++)
    {
        for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
        {
            GCEraseTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Erase_TR_Queue", xmlwriter);
        }
    }

    xmlwriter.Write_close_tag();
}


//下面这个函数不仅将事务放入对应的芯片队列中，并且按照通道及芯片的顺序调度这些芯片上的请求
void TSU_Priority_OutOfOrder::Schedule()
{
    opened_scheduling_reqs--;
    if (opened_scheduling_reqs > 0)        //意味着还有未完成的调度请求，需要等待这些请求完成后再继续处理
    {
        return;
    }

    if (opened_scheduling_reqs < 0)       
    {
        PRINT_ERROR("TSU_Priority_OutOfOrder: Illegal status!");
    }

    if (transaction_receive_slots.size() == 0)
    {
        return;
    }

    for (std::list<NVM_Transaction_Flash *>::iterator it = transaction_receive_slots.begin();
         it != transaction_receive_slots.end();
         it++)
    {//下面这段是把对应事务放入对应通道及芯片的队列中
        switch ((*it)->Type)
        {
        case Transaction_Type::READ:
            switch ((*it)->Source)
            {
            case Transaction_Source_Type::CACHE:
            case Transaction_Source_Type::USERIO:
                if ((*it)->Priority_class != IO_Flow_Priority_Class::UNDEFINED)
                {
                    UserReadTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID][static_cast<int>((*it)->Priority_class)].push_back((*it));
                }
                else
                {
                    UserReadTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID][IO_Flow_Priority_Class::HIGH].push_back((*it));   //如果事务的优先级未定义，则默认放入高优先级队列
                }
                break;
            case Transaction_Source_Type::MAPPING:
                MappingReadTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
                break;
            case Transaction_Source_Type::GC_WL:
                GCReadTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
                break;
            default:
                PRINT_ERROR("TSU_OutOfOrder: unknown source type for a read transaction!")
            }
            break;
        case Transaction_Type::WRITE:
            switch ((*it)->Source)
            {
            case Transaction_Source_Type::CACHE:
            case Transaction_Source_Type::USERIO:
                if ((*it)->Priority_class != IO_Flow_Priority_Class::UNDEFINED)
                {
                    UserWriteTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID][static_cast<int>((*it)->Priority_class)].push_back((*it));
                }
                else
                {
                    UserWriteTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID][IO_Flow_Priority_Class::HIGH].push_back((*it));
                }
                break;
            case Transaction_Source_Type::MAPPING:
                MappingWriteTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
                break;
            case Transaction_Source_Type::GC_WL:
                GCWriteTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
                break;
            default:
                PRINT_ERROR("TSU_OutOfOrder: unknown source type for a write transaction!")
            }
            break;
        case Transaction_Type::ERASE:
            GCEraseTRQueue[(*it)->Address.ChannelID][(*it)->Address.ChipID].push_back((*it));
            break;
        default:
            break;
        }
    }
    //将每个通道及每个芯片都查询一遍，查看能否调度读、写、及擦除事务
    for (flash_channel_ID_type channelID = 0; channelID < channel_count; channelID++)
    {
        if (_NVMController->Get_channel_status(channelID) == BusChannelStatus::IDLE)//前提是channel的状态是IDLE
        {
            for (unsigned int i = 0; i < chip_no_per_channel; i++)//把通道上的所有chip都遍历一遍
            {
                NVM::FlashMemory::Flash_Chip *chip = _NVMController->Get_chip(channelID, Round_robin_turn_of_channel[channelID]);                       //用轮询的方式确定是通道上的哪一个chip
                //The TSU does not check if the chip is idle or not since it is possible to suspend a busy chip and issue a new command
                //无论chip状态如何都会执行service_read_transaction，因为就算chip正处于busy状态，也会suspend这个chip上的事务然后发送命令
                if (!service_read_transaction(chip))
                {
                    if (!service_write_transaction(chip))
                    {
                        service_erase_transaction(chip);
                    }
                }
                Round_robin_turn_of_channel[channelID] = (flash_chip_ID_type)(Round_robin_turn_of_channel[channelID] + 1) % chip_no_per_channel;        //更新轮询通道下一次访问的chip
                if (_NVMController->Get_channel_status(chip->ChannelID) != BusChannelStatus::IDLE)
                {
                    break;//跳出这层的芯片循环，开始下一个通道的循环
                }
            }
        }
    }
}

Flash_Transaction_Queue *TSU_Priority_OutOfOrder::get_next_read_service_queue(NVM::FlashMemory::Flash_Chip *chip)
{
    if (UserReadTRQueue[chip->ChannelID][chip->ChipID][IO_Flow_Priority_Class::URGENT].size() > 0)      //优先检查用户读队列
    {
        return &UserReadTRQueue[chip->ChannelID][chip->ChipID][IO_Flow_Priority_Class::URGENT];
    }

    int numberOfTries = 0;                                                                              //初始化尝试次数
    while(numberOfTries < IO_Flow_Priority_Class::get_scheduling_weight(IO_Flow_Priority_Class::HIGH))  //high值是4，说明这里会尝试4次
    {
        nextPriorityClassRead[chip->ChannelID][chip->ChipID] = IO_Flow_Priority_Class::to_priority((nextPriorityClassRead[chip->ChannelID][chip->ChipID] + 1) % IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS);
        if (nextPriorityClassRead[chip->ChannelID][chip->ChipID] == IO_Flow_Priority_Class::URGENT)
        {
            nextPriorityClassRead[chip->ChannelID][chip->ChipID] = IO_Flow_Priority_Class::HIGH;
        }
        if (nextPriorityClassRead[chip->ChannelID][chip->ChipID] == IO_Flow_Priority_Class::HIGH)
        {
            numberOfTries++;
            currentWeightRead[chip->ChannelID][chip->ChipID]--;
            if (currentWeightRead[chip->ChannelID][chip->ChipID] <= 0)
            {
                currentWeightRead[chip->ChannelID][chip->ChipID] = IO_Flow_Priority_Class::get_scheduling_weight(IO_Flow_Priority_Class::HIGH);
            }
        }
        if (IO_Flow_Priority_Class::get_scheduling_weight(nextPriorityClassRead[chip->ChannelID][chip->ChipID]) >= currentWeightRead[chip->ChannelID][chip->ChipID]
            && UserReadTRQueue[chip->ChannelID][chip->ChipID][nextPriorityClassRead[chip->ChannelID][chip->ChipID]].size() > 0)
        {
            return &UserReadTRQueue[chip->ChannelID][chip->ChipID][nextPriorityClassRead[chip->ChannelID][chip->ChipID]];
        }
    }

    return NULL;
}

//主要用于将sourceQueue1和sourceQueue2赋予相应优先级的读事务队列(这些事务都在同一个chip上，如果无法获取读请求队列，则返回flase)
bool TSU_Priority_OutOfOrder::service_read_transaction(NVM::FlashMemory::Flash_Chip *chip)      //注意sourceQueue是一个chip级别的事务队列
{
    Flash_Transaction_Queue *sourceQueue1 = NULL, *sourceQueue2 = NULL;                 //sourceQueue1存储最高优先级队列，sourceQueue2存储次高优先级队列

    //Flash transactions that are related to FTL mapping data have the highest priority
    if (MappingReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)                   //如果有FTL映射相关的事务，将优先处理，因为FTL映射相关的事务具有最高优先级
    {
        sourceQueue1 = &MappingReadTRQueue[chip->ChannelID][chip->ChipID];              //sourceQueue1指向读取事务队列，因为具有最高优先级
        if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip) && GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
        {
            sourceQueue2 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];               //如果GC处于紧急模式，且GCReadTRQueue
        }
        else
        {
            sourceQueue2 = get_next_read_service_queue(chip);                           //如果GC不处于紧急模式，且垃圾回收读取事务队列中没有事务，使用函数选择下一个次高优先级读取事务队列
        }
    }
    else if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip))
    {
        //If flash transactions related to GC are prioritzed (non-preemptive execution mode of GC), then GC queues are checked first
        if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
        {
            sourceQueue1 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
            sourceQueue2 = get_next_read_service_queue(chip);
        }
        else if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
        {
            return false;                                                               //如果GC写入事务队列中有事务，返回false表示不能服务当前读取事务，因为GC写入事务需要优先处理
        }
        else if (GCEraseTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
        {
            return false;                                                               //如果 GC 擦除事务队列中有事务，那么返回 false，表示不能服务当前的读取事务，因为 GC 擦除事务需要优先处理。
        }
        else
        {
            sourceQueue1 = get_next_read_service_queue(chip);
            if (sourceQueue1 == NULL)
            {
                return false;                                                           //如果没有找到合适的队列，则返回false，表示当前没有事务可以服务
            }
        }
    }
    else
    {                                                                                   //没有映射表映射事务，gc也不处于紧急模式                             
        //If GC is currently executed in the preemptive mode, then user IO transaction queues are checked first
        sourceQueue1 = get_next_read_service_queue(chip);
        if (sourceQueue1 != NULL)
        {
            if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
            {
                sourceQueue2 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
            }
        }
        else if (get_next_write_service_queue(chip) != NULL)                            //读队列没有合适的事务，检查写队列
        {
            return false;
        }
        else if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
        {
            sourceQueue1 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
        }
        else
        {
            return false;
        }
    }


    //下面这段代码决定是否要暂停当前chip上的写或擦除操作，如果需要暂停，则发出暂停命令，然后处理读事务。

    bool suspensionRequired = false;
    ChipStatus cs = _NVMController->GetChipStatus(chip);   //通过book里面记录的内容查看chip的状态
    switch (cs)
    {
    case ChipStatus::IDLE:
        break;
    case ChipStatus::WRITING:
        if (!programSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
        {
            return false;
        }
        if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < writeReasonableSuspensionTimeForRead)
        {
            return false;
        }
        suspensionRequired = true;
    case ChipStatus::ERASING:
        if (!eraseSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
        {
            return false;
        }
        if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < eraseReasonableSuspensionTimeForRead)
        {
            return false;
        }
        suspensionRequired = true;
    default:
        return false;
    }

    issue_command_to_chip(sourceQueue1, sourceQueue2, Transaction_Type::READ, suspensionRequired);   //传入的suspensionRequired表示是否同意挂起正在进行的操作

    return true;
}

Flash_Transaction_Queue *TSU_Priority_OutOfOrder::get_next_write_service_queue(NVM::FlashMemory::Flash_Chip *chip)
{
    if (UserWriteTRQueue[chip->ChannelID][chip->ChipID][IO_Flow_Priority_Class::URGENT].size() > 0)
    {
        return &UserWriteTRQueue[chip->ChannelID][chip->ChipID][IO_Flow_Priority_Class::URGENT];
    }

    int numberOfTries = 0;
    while (numberOfTries < IO_Flow_Priority_Class::get_scheduling_weight(IO_Flow_Priority_Class::HIGH))
    {
        nextPriorityClassWrite[chip->ChannelID][chip->ChipID] = IO_Flow_Priority_Class::to_priority((nextPriorityClassWrite[chip->ChannelID][chip->ChipID] + 1) % IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS);
        if (nextPriorityClassWrite[chip->ChannelID][chip->ChipID] == IO_Flow_Priority_Class::URGENT)
        {
            nextPriorityClassWrite[chip->ChannelID][chip->ChipID] = IO_Flow_Priority_Class::HIGH;
        }
        if (nextPriorityClassWrite[chip->ChannelID][chip->ChipID] == IO_Flow_Priority_Class::HIGH)
        {
            numberOfTries++;
            currentWeightWrite[chip->ChannelID][chip->ChipID]--;
            if (currentWeightWrite[chip->ChannelID][chip->ChipID] <= 0)
            {
                currentWeightWrite[chip->ChannelID][chip->ChipID] = IO_Flow_Priority_Class::get_scheduling_weight(IO_Flow_Priority_Class::HIGH);
            }
        }
        if (IO_Flow_Priority_Class::get_scheduling_weight(nextPriorityClassWrite[chip->ChannelID][chip->ChipID]) >= currentWeightWrite[chip->ChannelID][chip->ChipID]
            && UserWriteTRQueue[chip->ChannelID][chip->ChipID][nextPriorityClassWrite[chip->ChannelID][chip->ChipID]].size() > 0)
        {
            return &UserWriteTRQueue[chip->ChannelID][chip->ChipID][nextPriorityClassWrite[chip->ChannelID][chip->ChipID]];
        }
    }

    return NULL;
}

bool TSU_Priority_OutOfOrder::service_write_transaction(NVM::FlashMemory::Flash_Chip *chip)
{
    Flash_Transaction_Queue *sourceQueue1 = NULL, *sourceQueue2 = NULL;

    //If flash transactions related to GC are prioritzed (non-preemptive execution mode of GC), then GC queues are checked first
    if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip))
    {
        if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
        {
            sourceQueue1 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];
            sourceQueue2 = get_next_write_service_queue(chip);
        }
        else if (GCEraseTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
        {
            return false;
        }
        else
        {
            sourceQueue1 = get_next_write_service_queue(chip);
            if(sourceQueue1 == NULL)
            {
                return false;
            }
        }
    }
    else
    {
        //If GC is currently executed in the preemptive mode, then user IO transaction queues are checked first
        sourceQueue1 = get_next_write_service_queue(chip);
        if (sourceQueue1 != NULL)
        {
            if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
            {
                sourceQueue2 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];
            }
        }
        else if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
        {
            sourceQueue1 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];
        }
        else
        {
            return false;
        }
    }

    bool suspensionRequired = false;
    ChipStatus cs = _NVMController->GetChipStatus(chip);
    switch (cs)
    {
    case ChipStatus::IDLE:
        break;
    case ChipStatus::ERASING:
        if (!eraseSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
            return false;
        if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < eraseReasonableSuspensionTimeForWrite)
            return false;
        suspensionRequired = true;
    default:
        return false;
    }

    issue_command_to_chip(sourceQueue1, sourceQueue2, Transaction_Type::WRITE, suspensionRequired);

    return true;
}

bool TSU_Priority_OutOfOrder::service_erase_transaction(NVM::FlashMemory::Flash_Chip *chip)
{
    if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
    {
        return false;
    }

    Flash_Transaction_Queue *source_queue = &GCEraseTRQueue[chip->ChannelID][chip->ChipID];
    if (source_queue->size() == 0)
    {
        return false;
    }

    issue_command_to_chip(source_queue, NULL, Transaction_Type::ERASE, false);

    return true;
}
} // namespace SSD_Components
