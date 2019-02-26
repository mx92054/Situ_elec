#include "spd_comm.h"

extern short wReg[];

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//	@retval	None
//-------------------------------------------------------------------------------
void SpdQueueInit(SpeedValueQueue *svq)
{
    int i;

    svq->ptr_head = 0;
    svq->ptr_tail = 0;
    svq->lSum_ang = 0;
    svq->lSum_tim = 0;
    for (i = 0; i < SPD1_QUEUE_LEN; i++)
    {
        svq->queue_ang[i] = 0;
        svq->queue_tim[i] = 0;
    }
}

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//          val:插入隊列的值
//	@retval	None
//-------------------------------------------------------------------------------
void SpdQueueIn(SpeedValueQueue *svq, short ang, short tim)
{
    svq->lSum_ang += ang;
    svq->lSum_ang -= svq->queue_ang[svq->ptr_head];
    svq->lSum_tim += tim;
    svq->lSum_tim -= svq->queue_tim[svq->ptr_head];

    svq->queue_ang[svq->ptr_head] = ang;
    svq->queue_tim[svq->ptr_head] = tim;

    svq->ptr_head++;
    if (svq->ptr_head >= SPD1_QUEUE_LEN)
        svq->ptr_head = 0;
}

//-------------------------------------------------------------------------------
//	@brief	速度值計算隊列初始化
//	@param	svq:隊列指針
//	@retval	隊列中保存數據的平均值
//-------------------------------------------------------------------------------
short SpdQueueAvgVal(SpeedValueQueue *svq)
{
    if (svq->lSum_tim == 0)
        return 0;
    return svq->lSum_ang * 1000 / svq->lSum_tim;
}

/****************************************************************
 *	@brief	PID模块初始化
 *	@param	pPid模块指针
 *          no 参数起始地址
 *	@retval	None
 ****************************************************************/
void PIDMod_initialize(PID_Module *pPid, int no)
{
    pPid->pParaAdr = &wReg[no];

    pPid->vOutL1 = 0;
    pPid->vOutL2 = 0;
    pPid->sDeltaL1 = 0;
    pPid->sDeltaL2 = 0;
}

/****************************************************************
 *	@brief	PID模块参数检测
 *	@param	pPid模块指针
 *          no 参数起始地址
 *	@retval	None
 ****************************************************************/
void PIDMod_update_para(PID_Module *pPid)
{
    if (pPid->pParaAdr[0] > 100 || pPid->pParaAdr[0] < 0) //判断输入寄存器地址
        pPid->pParaAdr[0] = 0;
    if (pPid->pParaAdr[1] >= 200 || pPid->pParaAdr[1] < 100) //判断输出寄存器地址
        pPid->pParaAdr[1] = 199;

    if (pPid->pParaAdr[7] < 0)
        pPid->pParaAdr[7] = 0;
    if (pPid->pParaAdr[7] > 100)
        pPid->pParaAdr[7] = 100;

    if (pPid->pParaAdr[9] < 0 || pPid->pParaAdr[9] > 3)
        pPid->pParaAdr[9] = 0;
}

/****************************************************************
 *	@brief	PID模块计算
 *	@param	pPid模块指针
 *	@retval	None
 ****************************************************************/
void PIDMod_step(PID_Module *pPid)
{
    long int pid_u, pid_out;
    long int curDelta, tmp, val;

    if (pPid->pParaAdr[9] == 0)
        return;

    tmp = wReg[pPid->pParaAdr[0]];
    tmp = tmp * pPid->pParaAdr[2] / 100;
    curDelta = pPid->pParaAdr[3] - tmp; //当前偏差值

    pid_u = pPid->pParaAdr[4] * (curDelta - pPid->sDeltaL1 +
                                 pPid->pParaAdr[5] * pPid->sDeltaL1 +
                                 pPid->pParaAdr[6] * (curDelta - 2 * pPid->sDeltaL1 + pPid->sDeltaL2));
    pPid->sDeltaL2 = pPid->sDeltaL1;
    pPid->sDeltaL1 = curDelta;

    pid_out = pPid->vOutL1;
    if (pPid->pParaAdr[8] == 0) //根据作用方式确定是增量还是减量
        pid_out -= pid_u;
    else
        pid_out += pid_u;

    //输出值限幅，避免调节器饱和
    if (pid_out > PID_MAX_OUT)
        pid_out = PID_MAX_OUT;
    if (pid_out < -PID_MAX_OUT)
        pid_out = -PID_MAX_OUT;

    //输出限幅
    tmp = 0x8000 * pPid->pParaAdr[7] / 100 - 1;
    val = pid_out / 1000;

    if (val > tmp)
        val = tmp;
    if (val < -tmp)
        val = -tmp;

    //输出方式选择
    val &= 0x0000FFFF;
    wReg[pPid->pParaAdr[1]] = 0x8000 + val; //单回路PID

    if (pPid->pParaAdr[9] == 2)
        wReg[pPid->pParaAdr[1] + 1] = 0x8000 + val; //正向并联PID
    if (pPid->pParaAdr[9] == 3)
        wReg[pPid->pParaAdr[1] + 1] = 0x8000 - val; //反向并联PID

    pPid->vOutL2 = pPid->vOutL1;
    pPid->vOutL1 = pid_out;
}

/*------------------end of file------------------------*/
