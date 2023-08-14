#include "main.h"

PathInfoTypedef PathInforms[PATH_NUM];//·��
Pid_Typedef TrajectCorrectPID;//����PID
uint16_t PathDotIndex = 0;//·��ѡȡ

int count = 0;
/**********************************************************************************************************
*�� �� ��: PathInit
*����˵��: ·����ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void PathInit()
{		
		PathInforms[0].PathDotsInfoArray = NULL;
		PathInforms[0].PathDotsNum = 0;
		PathInforms[1].PathDotsInfoArray = path0;//ȥǰ��վ·��
		PathInforms[1].PathDotsNum = PATH_MAX0;//·������
		PathInforms[2].PathDotsInfoArray = path1;//��Ѳ����·��
		PathInforms[2].PathDotsNum = PATH_MAX1;//·������
		PathInforms[3].PathDotsInfoArray = path2;//����·��1
		PathInforms[3].PathDotsNum = PATH_MAX2;//·������
		PathInforms[4].PathDotsInfoArray = path3;//����·��2
		PathInforms[4].PathDotsNum = PATH_MAX3;//·������
		PathInforms[5].PathDotsInfoArray = path4;//����·��3
		PathInforms[5].PathDotsNum = PATH_MAX4;//·������
		PathInforms[6].PathDotsInfoArray = path5;
		PathInforms[6].PathDotsNum = PATH_MAX5;//·������
		PathInforms[7].PathDotsInfoArray = path6;
		PathInforms[7].PathDotsNum = PATH_MAX6;//·������
	
		//pid������ʼ��
		TrajectCorrectPID.P = 3.0f;
		TrajectCorrectPID.D = 0.0f;
		TrajectCorrectPID.OutMax = 1000.0f;
	
		PathDotIndex = 1;//·�ߵ�ǰ����
		
}


/**********************************************************************************************************
*�� �� ��: chassisNavigate
*����˵��: ���е���
*��    ��: ChassisInfo ������ṹ��  PathInfo ��Ҫ���е���������
*�� �� ֵ: ��
**********************************************************************************************************/
float navSpeed_Limit = 2000.0f; 
void chassisNavigate(ChassisState_t *ChassisInfo, PathInfoTypedef *PathInfo)
{
    float dx, dy;
    uint16_t PathDotIndexMAX;
    const NavigationPoints *PathDotsInfo = PathInfo->PathDotsInfoArray;

    volatile ChassisNavTypedef *pCurrentState = &(ChassisInfo->CurrentState);
    volatile ChassisNavTypedef *pAimState = &(ChassisInfo->AimState);
					
    // Navigate dots change count in one cycle
    uint8_t ChangeNavigateDotCount = 0;
    float NavigateVx, NavigateVy; //x,y������ٶ�
    static u8 last_num = 0;     //��һ�ε�·����
	
    if (last_num != ChassisInfo->NavigatePathNum)
    {
        //��һ��last_num=0;PathDotIndex��ֵΪ1;
        //֮��last_num =ChassisInfo->NavigatePathNum
        PathDotIndex = 1;
    }

    PathDotIndexMAX = PathInfo->PathDotsNum - 1;

    pAimState->X = PathDotsInfo[PathDotIndex].x;//x�����Ŀ��ֵ
    pAimState->Y = PathDotsInfo[PathDotIndex].y;//y�����Ŀ��ֵ
    pAimState->Alpha = PathDotsInfo[PathDotIndex].alpha; //ƫ��

    dx = PathDotsInfo[PathDotIndex].x - pCurrentState->X;//������Ŀ�귽���ƫ��ֵ
    dy = PathDotsInfo[PathDotIndex].y - pCurrentState->Y;

		//�ٶ����� mm/s
    if (PathDotsInfo[PathDotIndex - 1].v > navSpeed_Limit)
    {
        NavigateVx = navSpeed_Limit * arm_cos_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
        NavigateVy = navSpeed_Limit * arm_sin_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
    }
    else
    {
        NavigateVx = PathDotsInfo[PathDotIndex - 1].v * arm_cos_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
        NavigateVy = PathDotsInfo[PathDotIndex - 1].v * arm_sin_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
    }

		//�����ֵ������ԭ·�������Ƕ�Ϊ�۽ǣ�ǰ��·����
    while (dx * NavigateVx + dy * NavigateVy < 0.0f && PathDotIndex < PathDotIndexMAX) //�ٶ�ʸ����ָ����һ��ʸ���н��ж��Ƿ��������·���㣬
    {
        ChangeNavigateDotCount++;
        PathDotIndex++;
        dx = PathDotsInfo[PathDotIndex].x - pCurrentState->X;
        dy = PathDotsInfo[PathDotIndex].y - pCurrentState->Y;
        if (PathDotsInfo[PathDotIndex - 1].v > navSpeed_Limit)
        {
            NavigateVx = navSpeed_Limit * arm_cos_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
            NavigateVy = navSpeed_Limit * arm_sin_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
        }
        else
        {
            NavigateVx = PathDotsInfo[PathDotIndex - 1].v * arm_cos_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
            NavigateVy = PathDotsInfo[PathDotIndex - 1].v * arm_sin_f32(PathDotsInfo[PathDotIndex - 1].V_Theta);
        }
    }
    //Change Navigate Dot
    if (ChangeNavigateDotCount > 0) // if ChangeNavigateDotCount is more than two maybe a hazard. �������ο�������������  �е㲻��⣬���ǻ���¶�ε�
    {
        pAimState->X = PathDotsInfo[PathDotIndex].x;
        pAimState->Y = PathDotsInfo[PathDotIndex].y;
    }

    //Multi dot navigate
    if (PathDotIndex < PathDotIndexMAX) //navigate regulate
    {
        float foreNavigateX = PathDotsInfo[PathDotIndex - 1].x;
        float foreNavigateY = PathDotsInfo[PathDotIndex - 1].y;
        float postNavigateX = PathDotsInfo[PathDotIndex].x;
        float postNavigateY = PathDotsInfo[PathDotIndex].y;
        float TrajectoryDis;
        float NavigateIntervalX, NavigateIntervalY, NavigateIntervalDis;
        float aimVt, aimVtX, aimVtY;
        //Trajectory deviation correct
        NavigateIntervalX = postNavigateX - foreNavigateX;
        NavigateIntervalY = postNavigateY - foreNavigateY;
        NavigateIntervalDis = sqrt(NavigateIntervalX * NavigateIntervalX + NavigateIntervalY * NavigateIntervalY); //������� �������|AB|
        //distance of the two parallel line,using vector product??fh:not this
        //NOte:the result is signed,notes the reference original
        TrajectoryDis = -((pCurrentState->X - foreNavigateX) * NavigateIntervalY - (pCurrentState->Y - foreNavigateY) * NavigateIntervalX)
						/ NavigateIntervalDis; //�������|A'O|

        //tangential velocity compute
				TrajectCorrectPID.SetPoint = 0.0f;
        aimVt = PID_Calc(&TrajectCorrectPID, TrajectoryDis);

        //ƫ��У�����ٶ�У������ȫ����ֱ�ӷֽ⵽x��y����
        //the x  component aimVt aimVtX : aimVtX = aimVt * cos(V_Theta + pi/2)
        aimVtX = -aimVt * NavigateIntervalY / NavigateIntervalDis;

        //the y  component aimVt aimVtY : aimVtY = aimVt * sin(V_Theta + pi/2)
        aimVtY = +aimVt * NavigateIntervalX / NavigateIntervalDis;
        pAimState->Vx = aimVtX + NavigateVx;
        pAimState->Vy = aimVtY + NavigateVy;

        last_num = ChassisInfo->NavigatePathNum; //���ϵĴ���һ����·���ţ�����������ж��Ƿ��µ�·����
    }
    else //the last dot Ŀǰ��ִ�е���һ���ֱ��ֹͣ
    {
        pAimState->X=PathDotsInfo[PathDotIndexMAX].x;
        pAimState->Y=PathDotsInfo[PathDotIndexMAX].y;
				pAimState->Alpha=PathDotsInfo[PathDotIndexMAX].alpha;
				
        //PathDotIndex = 1;
				pAimState->Vx = 0;
				pAimState->Vy = 0;
				chassis.NAV_State = FINISHED;//״̬����
		//������״̬��������
		
    }
}






