#ifndef PATH_H_
#define PATH_H_

#define PATH_MAX0 517 //���·����  
#define PATH_MAX1 561
#define PATH_MAX2 374
#define PATH_MAX3 274
#define PATH_MAX4 508
#define PATH_MAX5 485
#define PATH_MAX6 339
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
extern const NavigationPoints path6[PATH_MAX6];
#endif
