#ifndef PATH_H_
#define PATH_H_

#define PATH_MAX0 527 //���·����  
#define PATH_MAX1 520
#define PATH_MAX2 155
#define PATH_MAX3 1
#define PATH_MAX4 1
#define PATH_MAX5 1334
/************************* Path *************************/
//������·����Ϣ
typedef struct
{
    float x;   //x����
    float y;    //y����
    float V_Theta;  //�ٶȷ���
    float alpha;    //����λ��
    float delta;  // ����
    float v;   //�ٶ�
    uint8_t preview;   //��ǰ�ͺ�
} NavigationPoints; 

//����·��������
typedef struct
{
    const NavigationPoints *PathDotsInfoArray; //point to the inception of pathI[PathDotsNum]
    uint16_t PathDotsNum;
} PathInfoTypedef;

extern const NavigationPoints path0[PATH_MAX0];
extern const NavigationPoints path1[PATH_MAX1];
extern const NavigationPoints path2[PATH_MAX2];
extern const NavigationPoints path3[PATH_MAX3];
extern const NavigationPoints path4[PATH_MAX4];
extern const NavigationPoints path5[PATH_MAX5];
#endif
