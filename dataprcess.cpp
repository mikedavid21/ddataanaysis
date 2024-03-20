//---------------------------------------------------------------------------
#include <vcl.h>
#pragma hdrstop
//---------------------------------------------------------------------------
#pragma argsused                                  
//---------------------------------------------------------------------------
#include <stdio.h>
#include <math.h>
#include <iostream.h>
#include <dos.h>
#include <stdlib.h>
#include <io.h>
#include "assert.h"
#include "conio.h"
#define RstreamNum 8   //���Դ������������Ϊ RstreamNum,��0��ʼ.
#define RparaNum  6000 //RstreamNum*1000  //���Դ�����ܲ�������

//---------------------------------------------------------------------------
 struct headstruct
      {
        //�޸�ͬ������Ϣ**********[10]���Դ���10��������*****************************
        int frdepth[RstreamNum],frlength[RstreamNum],frpersec[RstreamNum],buffn[RstreamNum];   // ������/����  ,��������,ÿ�볤����,��������С;
        int idnum[RstreamNum],id_sbit[RstreamNum],id_len[RstreamNum];   //ID��λ�ã� //ID�ֿ�ʼ��Чλ��//ID����Чλ����,���㿪ʼ����;
        int time_mode[RstreamNum];//0-- DM6 format; 1-- BCD code(High 8 bits); 2-- BCD  code (Low 8Bits);
        int stream,parameter_number[RstreamNum];      //��������������
      };
  struct headstruct heads;

  struct parastruct      //ÿ����������Ϣ
      {
        char name[RstreamNum][50];      // ������
        int  wordl[RstreamNum];         //�ֳ�
        char  kind[RstreamNum][20];          // У׼����
        //У׼����  poly(����ʽ)  hyper(˫����) segm(���) no(��) ������У׼(bool)  �ⲿ����У׼(userfun)
        /*      0: ��ͨУ׼
                1��������
                2��a*cod+b    // phy[0]=a; phy[1]=b; cod[0]=0: �޷�����
                                                   1: �з�����������Ϊ�����λ��
                3: a/cod+b    //phy[0]=a; phy[1]=b;
        */

        //  int  invert;     // �����Ƿ�ת 0�� No,  1: Yes
        int  fraddr[RstreamNum];  // ���ܺ�
        int  fraddr1[RstreamNum],fraddr2[RstreamNum],fraddr3[RstreamNum];

        int  wdaddr[RstreamNum];  //�ֺ�
        int  wdaddr1[RstreamNum],wdaddr2[RstreamNum],wdaddr3[RstreamNum];

        int  bit_start[RstreamNum][6];   //  ȡλʱ����ʼλ���磺 0��0��1��4
        int  bit_len[RstreamNum][6];     //  ȡλʱ�ĳ��ȣ�  �磺 12��8��4��4
        int  bit_object[RstreamNum][6];  //Ŀ��λ��

        int cyl[RstreamNum];     //���ܲ�����
        int lutn[RstreamNum];     //У�ߵ���
        int cod[RstreamNum][34];
        double phy[RstreamNum][34];

        //*�¼�*
        int word_interval[RstreamNum];// �ּ��
        double coefficient[RstreamNum][6];//poly(����ʽ)ϵ��
        double segment[RstreamNum][8],segment_point[5];//˫���ߣ�
        char code_type[RstreamNum][10];//ԭ������;
      };
  struct parastruct *para;
  struct parastruct extpar[1004]; //para������������в���������extparÿ����������������

  struct infstruct        //������Ϣ�ļ�
       {
        char SYSINPUT[20];          //@SYSINPUT@     //�����֣�SYSINPUT����ʾϵͳ�ض����룬��Щ��Ϣ�ǹ̶��ġ�
        char Planename[20];        //            //�ɻ������ַ���
        char Planeno[20];          //04             //�ɻ��ţ��ַ���
        char FlightDate[20];       //2002-11-24     //�������ڣ��ַ���
        char FlightNo[20];         //1              //���мܴΣ��ַ���
        char MeasureSystem[20];    //DM5            //����ϵͳ���ַ���
        char DataProperty[20];     //PCM            //�������ԣ��ַ���
        float Weight;              //-1             //�ɻ����������ʵ�ͣ�
        float Core;                //-1             //������ģ�ʵ�ͣ�
        char Hang[50];              //-1             //�����ң��ַ���
        float Temperature;         //-1             //���£�ʵ�ͣ�
        float Press;               //-1             //��ѹ��ʵ�ͣ�
        float Windspeed;           //-1             //���٣�ʵ�ͣ�
        float View;                //-1             //�ܼ��ȣ�ʵ�ͣ�
        char Ground[50];            //-1             //���أ��ַ���
        char Order[50];             //-1             //��Ŀ���ַ���
        char Testperson[20];        //-1             //Ա���ַ���
        char FlightDataFile[1000];   //-1             //�����ļ�����(������·��) ���ַ���
        char DataHeadfile[1000];     //-1             //У׼�ļ�����(������·��) ���ַ���
        char SubjectName[50];       //111            //���ݴ����Ŀ���ƣ��ַ���
        char UserID[20];            //ZZ             //���ݴ����û����ƣ��ַ���
        char ProcessDate[20];       //2007-06-13     //���ݴ������ڣ��ַ���
        char SYSINPUTEND[20];       //@SYSINPUTEND@  //�����֣�SYSINPUTEND���ض��������
        char PARINFOR[20];          //@PARINFOR@           //�����֣�ParInfor�����������Ϣ

        char Par_name[RparaNum][50];            //������(�ַ�)
        char Par_unit[RparaNum][20];            //��λ(�ַ�)
        float Par_UP[RparaNum];                //���ޣ�ʵ�ͣ�
        float Par_down[RparaNum];              //���ޣ�ʵ�ͣ�
        char Par_Group[RparaNum][15];                //������ͨ�����ַ���
        char Par_note[RparaNum][50];                //������ע���ַ���
        /*
        ANINDI&D_1 -1 -1     //������(�ַ�) ��λ(�ַ�) ���ޣ�ʵ�ͣ� ���ޣ�ʵ�ͣ� ������ͨ�����ַ��� ������ע���ַ��� ��ÿ����Ϣ�����Կո������
        -1
        * -1
        ANOR*B_1 -1 -1
        -1
        * -1
        LCMC@A_1 -1 -1
        -1
        * -1
        */
        char PARINFOREND[50];       //@PARINFOREND@                //�����֣�ParINforEND��������Ϣ����
        char TIMEINFO[20];          //@TIMEINFO@                   //�����֣�TimeInfo��ʱ����Ϣ

        //10 1 1 10 11 11 1   N��ʱ���
        //��ʼʱ��Сʱ�����ͣ� ��ʼʱ����ӣ����ͣ� ��ʼʱ���루���ͣ� ����ʱ��Сʱ�����ͣ� ����ʱ����ӣ����ͣ� ����ʱ���루���ͣ� �����ʣ����ͣ���ÿ����Ϣ�����Կո����
        //char Par_name[RparaNum][50];            //������(�ַ�)  ���˳��
        char Par_order[RparaNum][50];           //����˳��      �м�˳��
        int order[RparaNum];                        //����˳��λ��

        int BegH[100];
        int BegM[100];
        int BegS[100];
        int EndH[100];
        int EndM[100];
        int EndS[100];
        int Rate[100]; 
        int BegTime[100];   //�¼�
        int EndTime[100];   //�¼�

        char TIMEINFOREND[20];      //@TIMEINFOREND@  int TIMEINFOREND;             //�����֣�TIMEINFOREND��ʱ�����Ϣ����
        char SYSOUTPUT[20];         //@SYSOUTPUT@                  //�����֣�SYSOUTPUT��ϵͳ�ض������Ϣ
        //ZZ20070613DM5PCM161958.eng   //��������ļ����ƣ��ļ���ʽ���󣩣����ļ�����Ҫ�������һ��
        //ZZ20070613DM5PCM161958.cod
        //ZZ20070613DM5PCM161958.sta     .eng(������)��.cod(��ֵ)��.sta(����״̬)
        char FilePath[300];
        char SYSOUTPUTEND[20];     //@SYSOUTPUTEND@               //�����֣�SYSOUTPUTEND��ϵͳ�������
        char INPUT[20];            //@INPUT@                      //�����֣�Input��У׼�������룬�û����Ը�����Ҫ���ж���
        char INPUTEND[20];         //@INPUTEND@                   //��չ����
        char OUTPUT[20];           //@OUTPUT@                     //�����֣�INPUTEND���������
        char OUTPUTEND[20];        //@OUTPUTEND@                  //�����֣�Output��У׼����������û����Ը�����Ҫ���ж���
       };
  struct infstruct  infs;

//--------------------------------------------------------------------------------
 int par_count[RstreamNum],all_count; //ev,
 AnsiString head_fname;
 FILE *hp,*pf,*dp; // read one whole subframe into subcod[8192];
 int read_subf(int strea);
 Word subcod[64000];
 double lut( __int64  c,int numb,int stre);
 long us_sig(int,long);
 long us_sig1(int,long);
 long us_sig2(int,long);
 long us_sigx(int sign,long val) ;
 float fcs_32s(unsigned long fcs_vcod,int numb,int stre);
 float fcs_16s(unsigned short int fcs_vcod,int numb,int stre);
 void cal_time(__int64,__int64,__int64,int*);  //����UMA2000ʱ�����
//--------------------------------------------------------------------------------
 //*�¼�*
 int CheckBox2Checked=-1;
 int TimeId = -99 ;  //������־��
 int read_hea();
 int process(int streams); //����,ʱ�����,����������
 int read_infs();
 int unite();
 bool stop=false;
 float check[15]={0};     // check[9]��CGauge1->Progress�ĵ���;
                          // check[8]��  ���ݵ���ʵ�����ʱ��
                          // check[10];   ������Чʱ���
                          // check[11];   �����һ������ȡ�м��ļ��Ĵ�С
                          // check[12];    �����һ������ȡ�м��ļ��ĺϲ�ʵʱ��С
                          // check[13];   �ж�ԭ����������ϲ���ѭ��
                          // check[0] ;   �����ʼ��inf_time��ֵ
                          // check[1];    �ٷֱ�
                          // check[14];   ��ǰ����ʱ����ʾ
                          //check[6]      �жϵ�ǰʱ����Ƿ����������.
                          //check[2]     uma2000�ж�

 int Progress=0; //�ٷֱ�
 int timei;

 FILE *tfp[RstreamNum+1],*tfc[RstreamNum+1];   //RstreamNum����
 int hour,minute,second,messcond;
 float mestime;

 FILE *inf,*TReng,*TRcod,*TRsta;
 AnsiString  SaveFileNamePath,Dir,Tdir,TAeng,TAcod,TAsta,TempFileName;
 AnsiString TempData,TempDir;
 AnsiString tfs;
 using namespace std;
 int inf_paranumber,inf_time,i=0,j;    //����������ʱ���
 int inf_stream;
 int distributary[RstreamNum+1][1006],distribut[RstreamNum+1]={0}; //����
 int stream_no;
 int ctime[6];
 int ErroTimetable=100;
 int Singlestr=0;
 int streamsnum=0,filenum[RstreamNum]={0};  //�账��ľ�������;
 int checkinf_time;
 int StreamMaxCyl=0; //��������������������ʵ�����
 
 double result[RparaNum];
 int Nresult[RparaNum];
 int  process(int streams);   
//--------------------old--------------------
  int findp[1000]; //�������Ŀ��Դ��������������
  int sts,ets,extr;
  FILE *fp,*fc;
  int ext_count;
  int k,it,npfr;
  int findt=0;
  short  addr[515][1006],dd[515];
  short addr1[515][1006],addr2[515][1006],addr3[515][1006],dd1[515],dd2[515],dd3[515];
  int maxe,tsub,kz;
  __int64 vcod[RparaNum];
  float vphy[RparaNum];
  char pname[200];
   int MaxSingleParaNmu;
//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
/*********************************************************************************/
// 1.��ÿ����,��ʱ�����ԭʼ�����ļ�����ȡ��Ӧ����ֵ;
// 2.��ֻ��һ��������Ҫ�������,����ȡ����ʱֱ��д������ļ�;
// 3.��ȡ�Ĳ���ֵ�ļ�������Ϣ�������ʱ�ļ����µ���Ӧ���ļ���,һ�����е�����ʱ���;
// 4.�����ļ����кϲ�,ͬʱ�ϲ��������ļ�;
// 5.��ʱ��ζ��������ļ��ϲ�,����ͬʱ���;
// 6.����ļ�������������ͬʱ��ηֿ���,��ֵ����һ���ļ���.
// 7.2011.1.7 INF �ļ��쳣����
/***********************************************************************************/
 clrscr(); //�������� 
 cout<<"  ��PCM���������ݴ���ʼ......"<<endl;
 try
   {
   int nRetCode = 0;
   Dir=argv[1];
   if(Dir.Length() == 0)
        {
                cout<<"  ϵͳ�޷���λ��Ϣ�ļ�������ϵ����Ա......"<<endl;
                return  nRetCode;
        }
   Tdir=ChangeFileExt(Dir,""); //��ʱ�ļ�Ŀ¼
   SaveFileNamePath=ExtractFilePath(Tdir); //����ļ�Ŀ¼

   TAsta=ChangeFileExt(Tdir,".sta"); //��־�ļ�
   if((TRsta=fopen(TAsta.c_str(),"wt"))==NULL)
       {
         cout<<"  �޷�������־�ļ�������ϵ����Ա......" <<endl;
         return  nRetCode;
       }

   if (!DirectoryExists(SaveFileNamePath))
      {
        if (!CreateDir(SaveFileNamePath))
             {
                cout<<"  Cannot create SaveFileNamePath directory,Please contact the administrator."<<endl;
                fprintf(TRsta,"%d\n",0);
                fprintf(TRsta,"%s\n","Cannot create SaveFileNamePath directory");
                fprintf(TRsta,"%s","Eof");
                return nRetCode;
             }
      }
   if (!DirectoryExists(Tdir))
    {
       if (!CreateDir(Tdir))
          {
                cout<<"  Cannot create TempDir directory,Please contact the administrator."<<endl;
                fprintf(TRsta,"%d\n",0);
                fprintf(TRsta,"%s\n","Cannot create TempDir directory");
                fprintf(TRsta,"%s","Eof");
                return nRetCode;
          }
     } 
   TempFileName=Tdir+"\\"+ExtractFileName(Tdir);
   if(read_infs()==0)
        {
              cout<<"  ��Ϣ�ļ������޷���������ϵ����Ա......"<<endl;
              fprintf(TRsta,"%d\n",0);
              fprintf(TRsta,"%s","��Ϣ�ļ������޷�����");
              fprintf(TRsta,"%s","Eof");
              return nRetCode;
        }
   cout<<"  ��Ϣ�ļ��������.....\n"<<endl;
   if(read_hea()==0){
              cout<<"  У���ļ������޷����أ�����ϵ����Ա......"<<endl;
              fprintf(TRsta,"%d\n",0);
              fprintf(TRsta,"%s\n","У���ļ������޷�����");
              fprintf(TRsta,"%s","Eof");
              return nRetCode;
              }
   cout<<"  У���ļ��������.....\n"<<endl;

   int MaxCyl=0;
   //���账��Ĳ��������������ʵ���ѡ������Ϊ�ϲ������ı�׼ʱ���� 
   for(j=0;j<streamsnum;j++)
          {
                if(MaxCyl<heads.frdepth[filenum[j]]*heads.frpersec[filenum[j]])
                      {
                           MaxCyl=heads.frdepth[filenum[j]]*heads.frpersec[filenum[j]];
                           StreamMaxCyl=filenum[j];
                      }
          } 
    //----------------------------output----------------------------------------
   TAeng=ChangeFileExt(Tdir,".eng");
   if((TReng=fopen(TAeng.c_str(),"wb"))==NULL)
       {
         cout<<"  �޷������������ļ�������ϵ����Ա......" <<endl;
         fprintf(TRsta,"%d\n",0);
         fprintf(TRsta,"%s\n","�޷������������ļ�");
         fprintf(TRsta,"%s","Eof");
         return  nRetCode;
       }

   TAcod=ChangeFileExt(Tdir,".cod");
   if((TRcod=fopen(TAcod.c_str(),"wb"))==NULL)
       {
         cout<<"  �޷�������ֵ�ļ�������ϵ����Ա......" <<endl;
         fprintf(TRsta,"%d\n",0);
         fprintf(TRsta,"%s\n","�޷�������ֵ�ļ�");
         fprintf(TRsta,"%s","Eof");
         return  nRetCode;
       }

  fwrite(&infs.Planename,sizeof(char)*20,1,TRcod);        //            //�ɻ������ַ���
  fwrite(&infs.Planeno,sizeof(char)*20,1,TRcod);          //04             //�ɻ��ţ��ַ���
  fwrite(&infs.FlightDate,sizeof(char)*20,1,TRcod);       //2002-11-24     //�������ڣ��ַ���
  fwrite(&infs.FlightNo,sizeof(char)*20,1,TRcod);         //1              //���мܴΣ��ַ���
  fwrite(&infs.SubjectName,sizeof(char)*50,1,TRcod);       //111            //���ݴ����Ŀ���ƣ��ַ���
  fwrite(&infs.MeasureSystem,sizeof(char)*20,1,TRcod);    //DM5            //����ϵͳ���ַ���
  fwrite(&infs.DataProperty,sizeof(char)*20,1,TRcod);     //PCM            //�������ԣ��ַ���
    infs.Weight=10000;
    fwrite(&infs.Weight,sizeof(float),1, TRcod);     //�ɻ�����������ַ���

  fwrite(&infs.Core,sizeof(infs.Core),1,TRcod);                //-1             //������ģ�ʵ��
  fwrite(&infs.Hang,sizeof(char)*50,1,TRcod);              //-1             //�����ң��ַ���
  fwrite(&infs.Temperature,sizeof(infs.Temperature),1,TRcod);         //-1             //���£�ʵ�ͣ�
  fwrite(&infs.Press,sizeof(infs.Press),1,TRcod);               //-1             //��ѹ��ʵ�ͣ�
  fwrite(&infs.Windspeed,sizeof(infs.Windspeed),1,TRcod);           //-1             //���٣�ʵ�ͣ�
  fwrite(&infs.View,sizeof(infs.View),1,TRcod);                //-1             //�ܼ��ȣ�ʵ�ͣ�
  fwrite(&infs.Ground,sizeof(char)*50,1,TRcod);            //-1             //���أ��ַ���
  fwrite(&infs.Order,sizeof(char)*50,1,TRcod);             //-1             //��Ŀ���ַ���
  fwrite(&infs.Testperson,sizeof(char)*20,1,TRcod);        //-1             //Ա���ַ���


        fwrite(&infs.UserID,sizeof(char)*20,1,TRcod);        //���ݴ����û����ƣ��ַ���
        fwrite(&infs.ProcessDate,sizeof(char)*50,1,TRcod);        //���ݴ�������

  fwrite(&inf_paranumber,sizeof(int),1,TRcod);         //����������
  for(i=0;i<inf_paranumber;i++) //������Ϣ
        {
                fwrite(&infs.Par_name[i],sizeof(char)*50,1,TRcod);     //������(�ַ�)
                fwrite(&infs.Par_unit[i],sizeof(char)*20,1,TRcod);     //��λ(�ַ�)
                fwrite(&infs.Par_UP[i],sizeof(float),1,TRcod);
                fwrite(&infs.Par_down[i],sizeof(float),1,TRcod);
                fwrite(&infs.Par_Group[i],sizeof(char)*15,1,TRcod);
        }

  fwrite(&infs.Planename,sizeof(char)*20,1,TReng);        //            //�ɻ������ַ���
  fwrite(&infs.Planeno,sizeof(char)*20,1,TReng);          //04             //�ɻ��ţ��ַ���
  fwrite(&infs.FlightDate,sizeof(char)*20,1,TReng);       //2002-11-24     //�������ڣ��ַ���
  fwrite(&infs.FlightNo,sizeof(char)*20,1,TReng);         //1              //���мܴΣ��ַ���
  fwrite(&infs.SubjectName,sizeof(char)*50,1,TReng);       //111            //���ݴ����Ŀ���ƣ��ַ���
  fwrite(&infs.MeasureSystem,sizeof(char)*20,1,TReng);    //DM5            //����ϵͳ���ַ���
  fwrite(&infs.DataProperty,sizeof(char)*20,1,TReng);     //PCM            //�������ԣ��ַ���
    infs.Weight=10000;
    fwrite(&infs.Weight,sizeof(float),1, TReng);     //�ɻ�����������ַ���
  fwrite(&infs.Core,sizeof(infs.Core),1,TReng);                //-1             //������ģ�ʵ��
  fwrite(&infs.Hang,sizeof(char)*50,1,TReng);              //-1             //�����ң��ַ���
  fwrite(&infs.Temperature,sizeof(infs.Temperature),1,TReng);         //-1             //���£�ʵ�ͣ�
  fwrite(&infs.Press,sizeof(infs.Press),1,TReng);               //-1             //��ѹ��ʵ�ͣ�
  fwrite(&infs.Windspeed,sizeof(infs.Windspeed),1,TReng);           //-1             //���٣�ʵ�ͣ�
  fwrite(&infs.View,sizeof(infs.View),1,TReng);                //-1             //�ܼ��ȣ�ʵ�ͣ�
  fwrite(&infs.Ground,sizeof(char)*50,1,TReng);            //-1             //���أ��ַ���
  fwrite(&infs.Order,sizeof(char)*50,1,TReng);             //-1             //��Ŀ���ַ���
  fwrite(&infs.Testperson,sizeof(char)*20,1,TReng);        //-1             //Ա���ַ���
       
        fwrite(&infs.UserID,sizeof(char)*20,1,TReng);        //���ݴ����û����ƣ��ַ���
        fwrite(&infs.ProcessDate,sizeof(char)*50,1,TReng);        //���ݴ�������

  fwrite(&inf_paranumber,sizeof(int),1,TReng);         //����������
  for(int i=0;i<inf_paranumber;i++)
        {
                fwrite(&infs.Par_name[i],sizeof(char)*50,1,TReng);     //������(�ַ�)
                fwrite(&infs.Par_unit[i],sizeof(char)*20,1,TReng);     //��λ(�ַ�)
                fwrite(&infs.Par_UP[i],sizeof(float),1,TReng);
                fwrite(&infs.Par_down[i],sizeof(float),1,TReng);
                fwrite(&infs.Par_Group[i],sizeof(char)*15,1,TReng);
        }

   //---------------------------process-----------------------------------------
   for( stream_no=0;stream_no<RstreamNum;stream_no++) //����������
        if( distribut[stream_no]>0 )
                {
                        if( process(stream_no)==0 )
                                {
                                        cout<<"  ��ȡ���ݴ�������ϵ����Ա......"<<endl;
                                        fprintf(TRsta,"%d\n",0);
                                        fprintf(TRsta,"%s\n","��ȡ���ݴ���.");
                                        fprintf(TRsta,"%s","Eof");
                                        return  nRetCode;
                                }
                }
   //----------------------------unite------------------------------------------
  //for(timesect=0;timesect<inf_time;timesect++){//��ʱ��δ���ϲ�
     if(Singlestr!=1)
        if(unite()==0 )
                {
                        cout<<"  �ϲ����ݴ���,����ϵ����Ա......"<<endl;
                        fprintf(TRsta,"%d\n",0);
                        fprintf(TRsta,"%s\n","�ϲ����ݴ���.");
                        fprintf(TRsta,"%s","Eof");
                        return nRetCode;
                }
//---------------------------------------------------------------------------

   fprintf(TRsta,"%d\n",1);
   fprintf(TRsta,"%s\n","���ݴ���ɹ�����.");
   fprintf(TRsta,"%s","Eof");

   fclose(TReng);
   fclose(TRcod);
   fclose(TRsta);

    //clreol();
    // //c+printf("  Current percentage is: %d%\r", 100);
   cout<<"\n  ���ݴ���ɹ�����......"<<endl;

   delete[] para;
   RemoveDir(Tdir);
   return nRetCode;
 }
 catch( ... )
 {
   cout<<"   �����쳣����������ϵ����Ա....."<<endl;
   exit(1);
 }

}
//------------------------------------------------------------------------------
int process(int streams)//����,����������
{
  try{
    AnsiString askk,asfp,asfc;
    TempData=infs.FlightDataFile;
    TempDir=ExtractFileExt(TempData);
    int  k1=0;
    if(streams!=0)
            {
                    askk=IntToStr(streams+1);
                    askk="_"+askk;
                    TempData=ChangeFileExt(TempData,askk)+TempDir;                                                                 
            }
     //clreol();
    cout<<"  ������ȡ��"<<(streams+1)<<"��������,���Ժ�........................"<<endl;
     // //c+printf("  Current percentage is: %d%\r", Progress);
    if((dp=fopen(TempData.c_str(),"rb"))==NULL)
            {                                                                                                                      
                    cout<<"  �޷�����ԭʼ����" <<(streams+1)<<"������ϵ����Ա......"<<endl;
                    return 0;                                                                                                      
            }
    char timess[4][14]={"HOUR_","MINUTE_","SECOND_","MSSECOND_"};
    for(i=0;i<par_count[streams];i++)                                                                                              
            {                                                                                                                      
                    for(j=0;j<4;j++)
                            {                                                                                                      
                                    askk=timess[j];
                                    askk=askk+IntToStr(streams+1);                                                                   
                                    if(stricmp(askk.c_str(),UpperCase( para[i].name[streams]).c_str())==0)
                                            {
                                                    extpar[j]=para[i];                                                             
                                                    findt++;
                                            }
                            }                                                                                                      
            }                                                                                                                      
    if(findt<3)
            {                                                                                                                      
                    cout<<"  ��" <<(streams+1)<< "�����ļ���" "ʱ�������û���ҵ�,����ϵ����Ա......"<<endl;
                    return 0;
            }                                                                                                                      
    findt=0;

    //clreol();
   cout<<"  ����У���账�����......"<<endl;
    // //c+printf("  Current percentage is: %d%\r", Progress);

   for(j=0;j<distribut[streams];j++)
        findp[j]=0;
   for(i=0;i<par_count[streams];i++) //�������в�������
        {
                for(j=0;j<distribut[streams];j++) //ʵ����ȡ�ĵ�����������
                        {
                                strcpy(pname,infs.Par_name[distributary[streams][j]]);
                                if(stricmp(pname,para[i].name[streams])==0)
                                        {
                                                extpar[j+4]=para[i];
                                                findp[j]=1;
                                        }
                        }
        }  
     for(i=0;i<distribut[streams];i++)
        if(findp[i]==0)
                {
                        cout<<"  ����"<<infs.Par_name[distributary[streams][i]]<<" û���ҵ�������ϵ����Ա......"<<endl;
                        return 0;
                }

   ext_count=distribut[streams]; //����ʵ�ʴ����������
  //--------------------------------star processing-----------------------------
  /****************************************************************************
   ÿ��frame_no���ܺź� word_no�ֺž��������������ȥ
   addr��addr1��addr2��addr3Ϊ�ܹ��ĸ��ֵ���ȡ��
                            Ϊ��һ��������subcod�ľ���λ��
  ****************************************************************************/ 
   for(i=0;i<515;i++)
      for(j=0;j<1006;j++)
        {
                addr[i][j]=0;
                addr1[i][j]=0;
                addr2[i][j]=0;
                addr3[i][j]=0;
        }   
   for(i=0;i<ext_count+4;i++)
     if(extpar[i].cyl[streams]<=heads.frdepth[streams])  // ��������/�Ӳ�������
       {
                it=1;
                addr[it][i]=extpar[i].fraddr[streams]*heads.frlength[streams]+extpar[i].wdaddr[streams];
                addr1[it][i]=extpar[i].fraddr1[streams]*heads.frlength[streams]+extpar[i].wdaddr1[streams];
                addr2[it][i]=extpar[i].fraddr2[streams]*heads.frlength[streams]+extpar[i].wdaddr2[streams];
                addr3[it][i]=extpar[i].fraddr3[streams]*heads.frlength[streams]+extpar[i].wdaddr3[streams];

                for(j=0;j<extpar[i].cyl[streams]-1;j++)
                        {
                                it++;
                                addr[it][i]=addr[it-1][i]+heads.frdepth[streams]*heads.frlength[streams]/extpar[i].cyl[streams];
                                addr1[it][i]=addr1[it-1][i]+heads.frdepth[streams]*heads.frlength[streams]/extpar[i].cyl[streams];
                                addr2[it][i]=addr2[it-1][i]+heads.frdepth[streams]*heads.frlength[streams]/extpar[i].cyl[streams];
                                addr3[it][i]=addr3[it-1][i]+heads.frdepth[streams]*heads.frlength[streams]/extpar[i].cyl[streams];
                        }
                addr[0][i]=extpar[i].cyl[streams];
       }
     else  //���ɲ���
       {
                npfr=extpar[i].cyl[streams]/heads.frdepth[streams];   //����ÿ���ܳ��ִ���
                it=1;
                for(j=0;j<heads.frdepth[streams];j++)
                        {
                                addr[it][i]=j*heads.frlength[streams]+extpar[i].wdaddr[streams];
                                addr1[it][i]=j*heads.frlength[streams]+extpar[i].wdaddr1[streams];
                                addr2[it][i]=j*heads.frlength[streams]+extpar[i].wdaddr2[streams];
                                addr3[it][i]=j*heads.frlength[streams]+extpar[i].wdaddr3[streams];

                                for(k=0;k<npfr-1;k++)
                                        {
                                                it++;
                                                addr[it][i]=addr[it-1][i]+heads.frlength[streams]/npfr;
                                                addr1[it][i]=addr1[it-1][i]+heads.frlength[streams]/npfr;
                                                addr2[it][i]=addr2[it-1][i]+heads.frlength[streams]/npfr;
                                                addr3[it][i]=addr3[it-1][i]+heads.frlength[streams]/npfr;
                                        }
                                it++;
                        }
                addr[0][i]=extpar[i].cyl[streams];
        } //end of for else


   //����ַ����addr���а������ֵ���롣
   maxe=0;
   for(i=0;i<ext_count+4;i++)
     if(addr[0][i]>maxe) maxe=addr[0][i]; //maxΪ���账���������������
   for(i=0;i<ext_count+4;i++)
      {
        tsub=addr[0][i];// ÿ�������Ĳ�����
        if(tsub!=0){
                kz=maxe/tsub;
           }
        else{
                kz=0;
           }
        for(j=0;j<tsub;j++)
                {
                        dd[j]=addr[j+1][i];
                        dd1[j]=addr1[j+1][i];
                        dd2[j]=addr2[j+1][i];
                        dd3[j]=addr3[j+1][i];
                }
        for(j=0;j<tsub;j++)
                for(k=kz*j+1;k<kz*(j+1)+1;k++)
                        {
                                addr[k][i]=dd[j];        //���㣬������������
                                addr1[k][i]=dd1[j];
                                addr2[k][i]=dd2[j];
                                addr3[k][i]=dd3[j];
                        }
      }
      for(i=0;i<ext_count+4;i++) addr[0][i]=maxe;
    //seaching the begin time.
  int wadd,wadd1,sv,sv_error_start,second_mf;
  __int64    maskw,cd1,cd2,cd3,cd4,cdd,cdd1,cdd2,cdd3,cdd4; //4�ֽ�,32λ 0~4294967295
  checkinf_time=inf_time;
  __int64 tc1,tc2,tc3;

  
   //clreol();
  cout<<"  ���ڲ���ʱ��................."<<endl;
   // //c+printf("  Current percentage is: %d%\r", Progress);

  if(Singlestr==1)
        {
                fp=TReng;
                if(CheckBox2Checked==0)
                        fc=TRcod;
        }
  else {
                askk=".eng_tmp"+IntToStr(streams);
                asfp=ChangeFileExt(TempFileName,askk);//������
                fp=fopen(asfp.c_str(),"w");
                askk=".cod_tmp"+IntToStr(streams);
                asfc=ChangeFileExt(TempFileName,askk);//ԭ��
                if(CheckBox2Checked==0)
                        fc=fopen(asfc.c_str(),"w");
        }
  int  CheckFseek=100;
  //---------------��ʱ��δ���-------------------
  for(int timetab=0;timetab<inf_time;timetab++)
        {
                                        //��ʱ�����ȡ
                                        //infs.BegTime[inf_i],infs.EndTime[inf_i],infs.Rate[inf_i],
                                        //��ʼʱ��,����ʱ��,������,
                if( inf_time-checkinf_time!=0 && Singlestr ==1 ) //����������ʾ��
                        {
                        fwrite(&TimeId,4,1,fp);
                        fwrite(&TimeId,4,1,fp);
                        fwrite(&TimeId,4,1,fp);
                        fwrite(&TimeId,4,1,fp);

                        if(CheckBox2Checked==0)
                                {
                                        fwrite(&TimeId,4,1,fc);
                                        fwrite(&TimeId,4,1,fc);
                                        fwrite(&TimeId,4,1,fc);
                                        fwrite(&TimeId,4,1,fc);
                                }
                        }
                checkinf_time=checkinf_time-1;
                
                if( Singlestr == 1 )    //����ֱ�����
                        {
                                fwrite(&infs.BegH[timetab],4,1,fp);
                                fwrite(&infs.BegM[timetab],4,1,fp);
                                fwrite(&infs.BegS[timetab],4,1,fp);
                                fwrite(&infs.EndH[timetab],4,1,fp);
                                fwrite(&infs.EndM[timetab],4,1,fp);
                                fwrite(&infs.EndS[timetab],4,1,fp);
                                fwrite(&infs.Rate[timetab],4,1,fp);

                                if(CheckBox2Checked==0){
                                        fwrite(&infs.BegH[timetab],4,1,fc);
                                        fwrite(&infs.BegM[timetab],4,1,fc);
                                        fwrite(&infs.BegS[timetab],4,1,fc);
                                        fwrite(&infs.EndH[timetab],4,1,fc);
                                        fwrite(&infs.EndM[timetab],4,1,fc);
                                        fwrite(&infs.EndS[timetab],4,1,fc);
                                        fwrite(&infs.Rate[timetab],4,1,fc);
                                        }
                        }
                CheckFseek=100;
                timei=0;
                sts=infs.BegTime[timetab];   //��ʼʱ��
                ets=infs.EndTime[timetab];   //����ʱ��
                extr=infs.Rate[timetab]; //������
                if(timetab>0)
                        if(infs.BegTime[timetab]<infs.EndTime[timetab-1])
                                rewind(dp);

                if( ets>check[8] && check[8]!=0 )
                        ets= check[8];

   //-------------process---------------------------------- 
        do{
                if(read_subf(streams))
                {
                        // time format process...
                        switch (heads.time_mode[streams])
                                {
                                        case 0:  // normal time format.
                                                wadd=addr[1][0];  // hour address
                                                ctime[0]=subcod[wadd]&0x0FFF;
                                                wadd=addr[1][1];  // minute address
                                                ctime[1]=subcod[wadd]&0x0FFF;
                                                wadd=addr[1][2];  // second address
                                                ctime[2]=subcod[wadd]&0x0FFF;
                                                 break;
                                        case 3: //�ش���ʽ
                                                wadd=addr[1][0];  // hour address
                                                cd1=(subcod[wadd]&0x0180)>>7;
                                                cd2=(subcod[wadd]&0x078)>>3;
                                                ctime[0]=cd1*10+cd2;
                                                wadd=addr[1][1]; wadd1=addr[1][0];  // minute, hour address
                                                cd1=(subcod[wadd1]&0x0007);
                                                cd2=(subcod[wadd]&0x0780)>>7;
                                                ctime[1]=cd1*10+cd2;
                                                wadd=addr[1][1];  // second address
                                                cd1=(subcod[wadd]&0x0070)>>4;
                                                cd2=(subcod[wadd]&0x000F);
                                                ctime[2]=cd1*10+cd2;
                                                break;
                                        case 4: //KAM-500��ʽ
                                                wadd=addr[1][0];  // TIME_Hi address
                                                cd1=(subcod[wadd]&0x1800)>>11;
                                                cd2=(subcod[wadd]&0x0780)>>7;
                                                ctime[0]=cd1*10+cd2;  // HOUR
                                                wadd=addr[1][0];    // TIME_HI address
                                                cd1=(subcod[wadd]&0x0070)>>4;
                                                cd2=(subcod[wadd]&0x000F);
                                                ctime[1]=cd1*10+cd2;  // MINUTE
                                                wadd=addr[1][1];  // TIME_LI address
                                                cd1=(subcod[wadd]&0xF000)>>12;
                                                cd2=(subcod[wadd]&0x0F00)>>8;
                                                ctime[2]=cd1*10+cd2;  // SECOND
                                                break;
                                        case 5: //770��ʽ
                                                wadd=addr[1][0];  // hour address
                                                cd1=(subcod[wadd]&0x0030)>>4;
                                                cd2=(subcod[wadd]&0x000F);
                                                ctime[0]=cd1*10+cd2;
                                                wadd=addr[1][1];   // minute address
                                                cd1=(subcod[wadd]&0xE000)>>13;
                                                cd2=(subcod[wadd]&0x1E00)>>9;
                                                ctime[1]=cd1*10+cd2;
                                                wadd=addr[1][1];  // minute address
                                                cd1=(subcod[wadd]&0x01C0)>>6;
                                                cd2=(subcod[wadd]&0x003C)>>2;
                                                ctime[2]=cd1*10+cd2;
                                                break;
                                          case 6: //����������ɼ���ʱ���ָ�ʽ    jqd
                                                // hour --> |_10H_|_1H_|_10M_|,   |4b|4b|4b|;
                                                // minute-> |_1M_|_10S_|_1S_|,    |4b|4b|4b|;
                                                //second -> |_100ms_|_10ms_|_1ms_|,|4b|4b|4b|;
                                                wadd=addr[1][0];  // hour address
                                                cd1=(subcod[wadd]&0x3000)>>12;
                                                cd2=(subcod[wadd]&0x0F00)>>8;
                                                ctime[0]=cd1*10+cd2;         //hour

                                                cd1=(subcod[wadd]&0x0070)>>4;
                                                cd2=(subcod[wadd]&0x000F);
                                                ctime[1]=cd1*10+cd2;         //minute

                                                wadd=addr[1][1];  // minute address
                                                cd1=(subcod[wadd]&0xE000)>>13;
                                                cd2=(subcod[wadd]&0x1E00)>>9;
                                                ctime[2]=cd1*10+cd2;       //second

                                                wadd=addr[1][2];  // mssecond address
                                                cd1=(subcod[wadd]&0xFFC0)>>6;
                                                ctime[3]=cd1;       //mssecond
                                                        //20140404 pgj

 /*       wadd=addr[k1+1][0];  // hour address

        cd1=(subcod[wadd]&0x00F0)>>4;
        cd2=(subcod[wadd]&0x000F);
        ctime[0]=cd1*10+cd2;         //hour

        wadd=addr[k1+1][1];   // minute address
        cd1=(subcod[wadd]&0xF000)>>12;
        cd2=(subcod[wadd]&0xF00)>>8;
        ctime[1]=cd1*10+cd2;         //minute

        wadd=addr[k1+1][1];  // minute address
        cd1=(subcod[wadd]&0x00F0)>>4;
        cd2=(subcod[wadd]&0x000F);
        ctime[2]=cd1*10+cd2;       //second
 */
 
                                                break;
                                        case 7: //UMA2000�ɼ���ʱ���ָ�ʽ
                                                // hour --> |1 lsb=655360ms|, |16b|;
                                                // minute-> |1 lsb=10 ms|,    |16b|;
                                                //second -> |1 lsb=1 Us|,     |16b|;
                                                wadd=addr[1][0];  // hour address
                                                cd1=(subcod[wadd]&0x0FFFF);
                                                tc1=cd1;
                                                cd1=(subcod[wadd+1]&0x0FFFF);
                                                tc2=cd1;
                                                cd1=(subcod[wadd+2]&0x0FFFF);
                                                tc3=cd1;
                                                cal_time(tc1,tc2,tc3,ctime);

                                                break;
                                        default:
                                               cout<<"��"<<streams<<"����ԭʼ���ݸ�ʽ����,����ϵ����Ա......"<<endl;
                                                return 0;
                                }
                if(ctime[0]>23) ctime[0]=0;
                if(ctime[1]>59) ctime[1]=0;
                if(ctime[2]>59) ctime[2]=0;

                sv=ctime[0]*3600+ctime[1]*60+ctime[2];


                if(ets<sv && CheckFseek==100)
                {
                  sv_error_start = sv;
                  CheckFseek=101;
                  continue;
                }

                if(sv > sv_error_start && sv-sv_error_start < 300)
                {
                  continue;
                }

                if(sv > sv_error_start && sv-sv_error_start >= 300)
                {
                  cout<<"��"<<timetab+1<<"���������ʱ��С�ڷ��п�ʼʱ�䣬�޷�����......"<<endl;
                  ErroTimetable=timetab;
                  goto lab1;//return 0;
                }


                if(sv>=sts && sv<=ets )
                {
                     break;   //��ǰʱ�������ȡ��ʼʱ�䣬ֱ�Ӵ���
                }


                }  //  if(read_subf()).....
        if(stop) return 0;
     }while( (sv!=sts) && (!feof(dp)) );
//---------------------------------------------------------
     if(sts>sv)
        {
                cout<<"  ��"<<(timetab+1)<<"��ʱ��εĴ���ʼʱ��������㣬�޷��������齫����Ŀ�ʼʱ�䶨Ϊ0:0:0......"<<endl;
                ErroTimetable=timetab;
                goto lab1;//return 0;
        }

     if(extr>heads.frpersec[streams]*maxe)
        {
            cout<<"  ��"<<(timetab+1)<<"��ʱ��ε���ȡ������̫��,����������ܲ����ʴ���......"<<endl;

         }
        int bch,sc;  //��BCH�����ܴ���һ��
        if(extr<=heads.frpersec[streams])
                {
                        bch=heads.frpersec[streams]/extr;
                        sc=bch;
                        maxe=1;
                }else{
                        bch=extr/heads.frpersec[streams];
                        bch=maxe/bch; sc=1;
                        if(bch<1) bch=1;
                }
//---------------------------------------------------------
        do{
             for(k1=0;k1<maxe;k1=k1+bch)
                {
                // time format process...
                    switch (heads.time_mode[streams])
                        {
                          case 0:  // normal time format.
                                wadd=addr[k1+1][0];  // hour address
                                ctime[0]=subcod[wadd]&0x0FFF;
                                wadd=addr[k1+1][1];  // minute address
                                ctime[1]=subcod[wadd]&0x0FFF;
                                wadd=addr[k1+1][2];  // second address
                                ctime[2]=subcod[wadd]&0x0FFF;
                                wadd=addr[k1+1][3];  // mssecond address
                                ctime[3]=subcod[wadd]&0x0FFF;
                                break;
                          case 3: //�ش���ʽ
                                wadd=addr[1][0];  // hour address
                                cd1=(subcod[wadd]&0x0180)>>7;
                                cd2=(subcod[wadd]&0x078)>>3;
                                ctime[0]=cd1*10+cd2;
                                wadd=addr[1][1]; wadd1=addr[1][0];  // minute, hour address
                                cd1=(subcod[wadd1]&0x0007);
                                cd2=(subcod[wadd]&0x0780)>>7;
                                ctime[1]=cd1*10+cd2;
                                wadd=addr[1][1];  // second address
                                cd1=(subcod[wadd]&0x0070)>>4;
                                cd2=(subcod[wadd]&0x000F);
                                ctime[2]=cd1*10+cd2;
                                break;
                         case 4: //KAM-500��ʽ
                                wadd=addr[k1+1][0];  // TIME_Hi address
                                cd1=(subcod[wadd]&0x1800)>>11;
                                cd2=(subcod[wadd]&0x0780)>>7;
                                ctime[0]=cd1*10+cd2;  // HOUR
                                wadd=addr[k1+1][0];    // TIME_HI address
                                cd1=(subcod[wadd]&0x0070)>>4;
                                cd2=(subcod[wadd]&0x000F);
                                ctime[1]=cd1*10+cd2;  // MINUTE
                                wadd=addr[k1+1][1];  // TIME_LI address
                                cd1=(subcod[wadd]&0xF000)>>12;
                                cd2=(subcod[wadd]&0x0F00)>>8;
                                ctime[2]=cd1*10+cd2;  // SECOND
                                wadd=addr[k1+1][1];  // TIME_LI address
                                cd1=(subcod[wadd]&0x00F0)>>4;
                                cd2=(subcod[wadd]&0x000F);
                                ctime[3]=cd1*100+cd2*10;  // MSSECOND
                                wadd=addr[k1+1][2];  // MICROTIME address
                                cd1=(subcod[wadd]&0xF000)>>12;
                                ctime[3]=ctime[3]+cd1;  // MSSECOND
                                break;
                        case 5: //770��ʽ
                                wadd=addr[k1+1][0];  // hour address
                                cd1=(subcod[wadd]&0x0030)>>4;
                                cd2=(subcod[wadd]&0x000F);
                                ctime[0]=cd1*10+cd2;
                                wadd=addr[k1+1][1];   // minute address
                                cd1=(subcod[wadd]&0xE000)>>13;
                                cd2=(subcod[wadd]&0x1E00)>>9;
                                ctime[1]=cd1*10+cd2;
                                wadd=addr[k1+1][1];  // minute address
                                cd1=(subcod[wadd]&0x01C0)>>6;
                                cd2=(subcod[wadd]&0x003C)>>2;
                                ctime[2]=cd1*10+cd2;
                                wadd=addr[k1+1][1];  // minute address
                                cd1=(subcod[wadd]&0x01C0)>>6;
                                cd2=(subcod[wadd]&0x003C)>>2;
                                ctime[2]=cd1*10+cd2;

                                wadd=addr[k1+1][2]; wadd1=addr[k1+1][1];  // second ,minute address
                                cd1=(subcod[wadd1]&0x0003);
                                cd2=(subcod[wadd]&0xC000)>>14;
                                cd1=cd1*4+cd2;
                                cd2=(subcod[wadd]&0x3C00)>>10;
                                cd3=(subcod[wadd]&0x03C0)>>6;
                                ctime[3]=cd1*100+cd2*10+cd3;
                                break;
                        case 6: //����������ɼ���ʱ���ָ�ʽ  jqd
                                // hour --> |_10H_|_1H_|_10M_|,   |4b|4b|4b|;
                                // minute-> |_1M_|_10S_|_1S_|,    |4b|4b|4b|;
                                //second -> |_100ms_|_10ms_|_1ms_|,|4b|4b|4b|;
                                wadd=addr[k1+1][0];  // hour address
                                cd1=(subcod[wadd]&0x3000)>>12;
                                cd2=(subcod[wadd]&0x0F00)>>8;
                                ctime[0]=cd1*10+cd2;         //hour

                                cd1=(subcod[wadd]&0x0070)>>4;
                                cd2=(subcod[wadd]&0x000F);
                                ctime[1]=cd1*10+cd2;         //minute

                                wadd=addr[k1+1][1];  // minute address
                                cd1=(subcod[wadd]&0xE000)>>13;
                                cd2=(subcod[wadd]&0x1E00)>>9;
                                ctime[2]=cd1*10+cd2;       //second

                                wadd=addr[k1+1][2];  // mssecond address
                                cd1=(subcod[wadd]&0xFFC0)>>6;
                                ctime[3]=cd1;       //mssecond
                                break;
                        case 7: //UMA2000�ɼ���ʱ���ָ�ʽ
                                // hour --> |1 lsb=655360ms|, |16b|;
                                // minute-> |1 lsb=10 ms|,    |16b|;
                                //second -> |1 lsb=1 Us|,     |16b|;
                                wadd=addr[k1+1][0];  // hour address
                                cd1=(subcod[wadd]&0x0FFFF);
                                tc1=cd1;
                                cd1=(subcod[wadd+1]&0x0FFFF);
                                tc2=cd1;
                                cd1=(subcod[wadd+2]&0x0FFFF);
                                tc3=cd1;
                                cal_time(tc1,tc2,tc3,ctime);

                               // check[2]=100;
                                break;
                        default:
                                cout<<"��"<<streams<<"����ԭʼ����ʱ���ʽ����,����ϵ����Ա......"<<endl;
                                return 0;
                    }
                if(ctime[0]>23) ctime[0]=0;
                if(ctime[1]>59) ctime[1]=0;
                if(ctime[2]>59) ctime[2]=0;

                sv=ctime[0]*3600+ctime[1]*60+ctime[2];   //sv ʱ�����ʱ��
                if(sv>=(ctime[0]*3600+ctime[1]*60+ctime[2])&& timei==0)
                        {
                                printf("  ����ʼʱ�䣺%02d:%02d:%02d\n\r",ctime[0],ctime[1],ctime[2]);
                                timei=ctime[0]*3600+ctime[1]*60+ctime[2];
                                check[14]=timei;
                        }
               if(sv>(check[14]+59+270))
                        {
                                printf("  ��ǰ����ʱ�䣺%02d:%02d:%02d\n\r",ctime[0],ctime[1],ctime[2]);
                                check[14]=ctime[0]*3600+ctime[1]*60+ctime[2];
                        }

                check[1]= Progress;
                Progress=(int) ( check[9]+( ctime[0]*3600+ctime[1]*60+ctime[2]- timei )*80.0/( ( ets - timei )*streamsnum*inf_time ) );

                if(stop) return 0;
                int l_bit,s_bit;
                int ppk=0;
                for(i=4;i<ext_count+4;i++)
                        {
                            wadd=addr[k1+1][i];
                            switch(extpar[i].wordl[streams])
                              {
                                case 1: //���ֲ���
                                        maskw=0;
                                        s_bit=extpar[i].bit_start[streams][0];
                                        l_bit=extpar[i].bit_start[streams][0]+extpar[i].bit_len[streams][0];
                                        for(j=s_bit;j<l_bit;j++)
                                                maskw=maskw+(short)pow(2,j);
                                        vcod[i]=(subcod[wadd]&maskw)>>s_bit;
                                        vphy[i]=lut(vcod[i],i,streams);
                                        break;
                                case 2://˫�ֲ���
                                        //word 1
                                         maskw=0;
                                        s_bit=extpar[i].bit_start[streams][0];
                                        l_bit=extpar[i].bit_start[streams][0]+extpar[i].bit_len[streams][0];
                                        for(j=s_bit;j<l_bit;j++)
                                                maskw=maskw+(short)pow(2,j);
                                        cd1=subcod[wadd];
                                        cd1=(cd1&maskw)>>s_bit;
                                        //word 2
                                        maskw=0;
                                        s_bit=extpar[i].bit_start[streams][1];
                                        l_bit=extpar[i].bit_start[streams][1]+extpar[i].bit_len[streams][1];
                                        for(j=s_bit;j<l_bit;j++)
                                                maskw=maskw+(short)pow(2,j);
                                        //cd2=subcod[wadd+1];
                                        cd2=subcod[ addr1[k1+1][i] ];
                                        cd2=(cd2&maskw)>>s_bit;

                                        if(stricmp(extpar[i].kind[streams],"rs232")==0)      //20091015����
                                                {
                                                        vcod[i]=cd1;
                                                        vphy[i]=(cd1*100+cd2)/100.0;
                                                        break;
                                                }

                                        if( (extpar[i].bit_object[streams][0]+extpar[i].bit_object[streams][1])==0  ) //��������û����д��Ŀ��λ��������ȡ
                                                cdd=cd1<<extpar[i].bit_len[streams][1];
                                        else
                                                {
                                                        cdd=cd1<<extpar[i].bit_object[streams][0]; //�����ƶ�  bit_object[0]λ:��ʼλ�� bit_object[0]λ
                                                        cd2=cd2<<extpar[i].bit_object[streams][1];
                                                }
                                        cdd=cdd|cd2;
                                        vcod[i]=cdd;
                                        if( stricmp(extpar[i].name[streams],"U13_2_1_3_2")==0 ) //2011.4.29�ɻ�����ӱ��
                                            {
                                               ppk=(cdd&0xFFFFFFFF)>>16;
                                               if( ppk==21764 )
                                                   { 
                                                       cdd= (vcod[i]&0xFFFF);
                                                       vphy[i]=lut(cdd,i,streams);
                                                   }
                                               break;
                                            }

                                       /* else if( stricmp(extpar[i].code_type[streams],"fcsf")==0 ) //fcs�ɿ�16λ�з���
                                             {
                                                   vphy[i]=fcs_16s((unsigned short int)vcod[i],i,streams);
                                                   break;
                                             }
                                        else if( stricmp(extpar[i].code_type[streams],"fcsf")==0 )//�ɿ�32λ�з��Ÿ���
                                                        vphy[i]=fcs_32s(vcod[i],i,streams);

                                        */
                                       if( stricmp(extpar[i].code_type[streams],"fcsf")==0 )
                                        {
                                           if(extpar[i].bit_len[streams][0]+extpar[i].bit_len[streams][1]>16)
                                                  vphy[i]=fcs_32s( (unsigned long int)vcod[i],i,streams);      //fcs�ɿ�32λ�з���
                                           else
                                                  vphy[i]=fcs_16s((unsigned short int)vcod[i],i,streams); //fcs�ɿ�16λ�з���
                                           break;
                                        } 
                                        vphy[i]=lut(vcod[i],i,streams);
                                        break;
                                case 3:
                                        //word 1
                                        maskw=0;
                                        s_bit=extpar[i].bit_start[streams][0];
                                        l_bit=extpar[i].bit_start[streams][0]+extpar[i].bit_len[streams][0];
                                        for(j=s_bit;j<l_bit;j++)
                                                maskw=maskw+(short)pow(2,j);
                                        cd1=(subcod[wadd]&maskw)>>s_bit;
                                        // word 2
                                        maskw=0;
                                        s_bit=extpar[i].bit_start[streams][1];
                                        l_bit=extpar[i].bit_start[streams][1]+extpar[i].bit_len[streams][1];
                                        for(j=s_bit;j<l_bit;j++)
                                                maskw=maskw+(short)pow(2,j);
                                        cd2=(subcod[ addr1[k1+1][i] ]&maskw)>>s_bit;
                                        // word 3
                                        maskw=0;
                                        s_bit=extpar[i].bit_start[streams][2];
                                        l_bit=extpar[i].bit_start[streams][2]+extpar[i].bit_len[streams][2];
                                        for(j=s_bit;j<l_bit;j++)
                                                maskw=maskw+(short)pow(2,j);
                                        cd3=(subcod[ addr2[k1+1][i] ]&maskw)>>s_bit;

                                        if( (extpar[i].bit_object[streams][0]+extpar[i].bit_object[streams][1]+extpar[i].bit_object[streams][2])==0  )
                                                {
                                                        cdd=cd1<<(extpar[i].bit_len[streams][1]+extpar[i].bit_len[streams][2]);
                                                        cdd1=cd2<<extpar[i].bit_len[streams][2];
                                                        cdd=cdd|cdd1; cdd=cdd|cd3;
                                                }
                                        else
                                                {
                                                        cdd=cd1<<extpar[i].bit_object[streams][0]; //�����ƶ�  bit_object[0]λ:��ʼλ�� bit_object[0]λ
                                                        cdd1=cd2<<extpar[i].bit_object[streams][1];
                                                        cdd2=cd3<<extpar[i].bit_object[streams][2];
                                                        cdd=cdd|cdd1; cdd=cdd|cdd2;
                                                }
                                        vcod[i]=cdd;

                                        if( stricmp(extpar[i].code_type[streams],"fcsf")==0 )
                                        {
                                           if(extpar[i].bit_len[streams][0]+extpar[i].bit_len[streams][1]+extpar[i].bit_len[streams][2]>16)
                                                  vphy[i]=fcs_32s( (unsigned long int)vcod[i],i,streams);      //fcs�ɿ�32λ�з���
                                           else
                                                  vphy[i]=fcs_16s((unsigned short int)vcod[i],i,streams); //fcs�ɿ�16λ�з���
                                           break;
                                        }
                                        vphy[i]=lut(vcod[i],i,streams);
                                        break;
                                case 4:
                                                //word 1
                                                maskw=0;
                                                s_bit=extpar[i].bit_start[streams][0];
                                                l_bit=extpar[i].bit_start[streams][0]+extpar[i].bit_len[streams][0];
                                                for(j=s_bit;j<l_bit;j++)
                                                        maskw=maskw+(short)pow(2,j);
                                                cd1=(subcod[wadd]&maskw)>>s_bit;
                                                // word 2
                                                maskw=0;
                                                s_bit=extpar[i].bit_start[streams][1];
                                                l_bit=extpar[i].bit_start[streams][1]+extpar[i].bit_len[streams][1];
                                                for(j=s_bit;j<l_bit;j++)
                                                        maskw=maskw+(short)pow(2,j);
                                                cd2=(subcod[ addr1[k1+1][i] ]&maskw)>>s_bit;
                                                // word 3
                                                maskw=0;
                                                s_bit=extpar[i].bit_start[streams][2];
                                                l_bit=extpar[i].bit_start[streams][2]+extpar[i].bit_len[streams][2];
                                                for(j=s_bit;j<l_bit;j++)
                                                        maskw=maskw+(short)pow(2,j);
                                                cd3=(subcod[ addr2[k1+1][i] ]&maskw)>>s_bit;
                                                // word 4
                                                maskw=0;
                                                s_bit=extpar[i].bit_start[streams][3];
                                                l_bit=extpar[i].bit_start[streams][3]+extpar[i].bit_len[streams][3];
                                                for(j=s_bit;j<l_bit;j++)
                                                        maskw=maskw+(short)pow(2,j);
                                                cd4=(subcod[ addr3[k1+1][i] ]&maskw)>>s_bit;

                                                cdd=0;
                                                if( (extpar[i].bit_object[streams][0]+extpar[i].bit_object[streams][1]+extpar[i].bit_object[streams][2]+extpar[i].bit_object[streams][3])==0  )
                                                        {
                                                                cdd1=cd1<<(extpar[i].bit_len[streams][1]+extpar[i].bit_len[streams][2]+extpar[i].bit_len[streams][3]);
                                                                cdd2=cd2<<(extpar[i].bit_len[streams][2]+extpar[i].bit_len[streams][3]);
                                                                cdd3=cd3<<extpar[i].bit_len[streams][3];
                                                                cdd=cdd|cdd1; cdd=cdd|cdd2;cdd=cdd|cdd3;cdd=cdd|cd4;
                                                        }
                                                else
                                                        {
                                                                cdd1=cd1<<extpar[i].bit_object[streams][0]; //�����ƶ�  bit_object[0]λ:��ʼλ�� bit_object[0]λ
                                                                cdd2=cd2<<extpar[i].bit_object[streams][1];
                                                                cdd3=cd3<<extpar[i].bit_object[streams][2];
                                                                cdd4=cd4<<extpar[i].bit_object[streams][3];
                                                                cdd=cdd|cdd1; cdd=cdd|cdd2;cdd=cdd|cdd3;cdd=cdd|cdd4;
                                                        }
                                                vcod[i]=cdd;
                                                if( stricmp(extpar[i].code_type[streams],"fcsf")==0 )
                                                {
                                                   if(extpar[i].bit_len[streams][0]+extpar[i].bit_len[streams][1]+extpar[i].bit_len[streams][2]+extpar[i].bit_len[streams][3]>16)
                                                        vphy[i]=fcs_32s( (unsigned long int)vcod[i],i,streams);      //fcs�ɿ�32λ�з���
                                                   else
                                                        vphy[i]=fcs_16s((unsigned short int)vcod[i],i,streams); //fcs�ɿ�16λ�з���
                                                   break;
                                                }
                                                vphy[i]=lut(vcod[i],i,streams);
                                        break;
                                default:
                                         vcod[i]=0;
                                         vphy[i]=0.0;
                                         cout<<extpar[i].name[streams]<<"  У�߲���λ��������󣬸ò�����������������ϵ����Ա......"<<endl;
                        }// end for switch(extpar[i].wordl) ....
                }// end of for for(i=0;i<ext_count;...
                if( Singlestr ==1 )
                        {
                                fwrite(&ctime[0],4,1,fp);
                                fwrite(&ctime[1],4,1,fp);
                                fwrite(&ctime[2],4,1,fp);
                                mestime= (float)ctime[3];
                                fwrite(&mestime,4,1,fp);
                                for(j=0;j<ext_count;j++)
                                        {
                                                result[j]=vphy[j+4];
                                                fwrite(&result[j],sizeof(double),1,fp);
                                        }
                                if(CheckBox2Checked==0)
                                        {
                                                fwrite(&ctime[0],4,1,fc);
                                                fwrite(&ctime[1],4,1,fc);
                                                fwrite(&ctime[2],4,1,fc);
                                                mestime= (float)ctime[3];
                                                fwrite(&mestime,4,1,fc);
                                                for(j=0;j<ext_count;j++)
                                                {
                                                        fwrite(&vcod[j+4],sizeof(int),1,fc);
                                                }
                                        }

                        }
                else
                        {

                                fprintf(fp,"%02d:%02d:%02d:%03d   ",ctime[0],ctime[1],ctime[2],ctime[3]);  //����ļ�
                                if(CheckBox2Checked==0)
                                        fprintf(fc,"%02d:%02d:%02d:%03d   ",ctime[0],ctime[1],ctime[2],ctime[3]);   //��ֵ�ļ�
                                for(j=0;j<ext_count;j++)
                                        {
                                                fprintf(fp," %30.5f",vphy[j+4]);
                                                if(CheckBox2Checked==0)
                                                        fprintf(fc," %30u",vcod[j+4]);
                                        }
                                fprintf(fp,"\n");
                                if(CheckBox2Checked==0)
                                        fprintf(fc,"\n");
                            
                        }
        }// end for for(k1=0;k1<maxe;k1=k1+bch.....
        for(j=0;j<sc;j++)
                if(!read_subf(streams))
                        goto lab1;
        }while(sv!=ets);    //(sv<=ets);//

        lab1:
        if( ErroTimetable!=timetab  ) //����ʵ��ʱ��εĲ����
                {
                        if( timetab!=inf_time-1 && Singlestr!=1)//���һ��ʱ��λ��ߵ����� �����
                                {
                                        fprintf(fp,"%3d:%3d:%3d:%3d\n",TimeId,TimeId,TimeId,TimeId);  //ʱ��ν�����־
                                        if(CheckBox2Checked==0)
                                                fprintf(fc,"%3d:%3d:%3d:%3d\n",TimeId,TimeId,TimeId,TimeId);   //ʱ��ν�����־
                                }
                        //gotoxy(wherex()-24, wherey()-1);
                         //clreol();
                        //cout<<"  �������ʱ�䣺" <<ctime[0]<< ":"<<ctime[1]<<":"<<ctime[2]<<endl;
                        printf("  �������ʱ�䣺%02d:%02d:%02d\n\r",ctime[0],ctime[1],ctime[2]);
                         // //c+printf("  Current percentage is: %d%\r", Progress);

                }
        Progress=(int) ( check[9]+ 80.0/( streamsnum*inf_time ) ); //ÿ������ÿ��ʱ��δ������ʱӦ�ôﵽ�İٷֱ�
        check[9]=check[9]+ 80.0/( streamsnum*inf_time );

        if(sv<ets)
                {
                        cout<<"  ��"<<(timetab+1)<<"��ʱ��������ļ�����,δ�������ʱ��......."<<endl;
                        if ( (sts-sv-10)*2*heads.frdepth[streams]*heads.frlength[streams]*heads.frpersec[streams]!=0 )
                                check[8]= ctime[0]*3600+ctime[1]*60+ctime[2];  //���ݵ���ʵ�����ʱ��
                }
        else
                {
                         //clreol();
                        cout<<"  ��"<<(timetab+1)<<"��ʱ���������ȡ����......"<<endl;
                         // //c+printf("  Current percentage is: %d%\r", Progress);
                }

  }//��ʱ��δ���
  if( Singlestr ==1 ) //����������ʾ��
        {
                fwrite(&TimeId,4,1,fp);
                fwrite(&TimeId,4,1,fp);
                fwrite(&TimeId,4,1,fp);
                fwrite(&TimeId,4,1,fp);

                if(CheckBox2Checked==0)
                        {
                                fwrite(&TimeId,4,1,fc);
                                fwrite(&TimeId,4,1,fc);
                                fwrite(&TimeId,4,1,fc);
                                fwrite(&TimeId,4,1,fc);
                        }

        }
  fclose(fp);
  if(CheckBox2Checked==0)
        fclose(fc);

//---------------------------��ʱ��δ���---------------------------------------
 fclose(dp);
 return 1;  
 }
 catch ( ... )
        {
                cout<<"  ��ȡ�����쳣,����ϵ����Ա......"<<endl;
                exit(1);
        }
  return 1;
}
//------------------------------------------------------------------------------
int unite()
 {
  // FILE *Test;
  // Test=fopen("c:\\test.txt","wt") ;
   try{
   int summation=0;
   k=0;

   char timestr[20];
   AnsiString askk,asfp,asfc;
   int DelStream,DelTimetable;
   int p=0 ;
   for(i=0;i<RstreamNum;i++)//��������ʱ����ļ�      RstreamNum
   {
     if(distribut[i]>0)
        {
                //cout<<"  У׼��" <<(i+1)<<"��������,���Ժ�......"<<endl;

                askk=".eng_tmp"+IntToStr(i);
                asfp=ChangeFileExt(TempFileName,askk);//������

                askk=".cod_tmp"+IntToStr(i);
                asfc=ChangeFileExt(TempFileName,askk);//ԭ��
                if((tfp[i]=fopen(asfp.c_str(),"rt+"))==NULL)
                        {
                                cout<<"  �޷���������������" <<(i+1)+ "������ϵ����Ա......"<<endl;
                                return 0;
                        }
                if(CheckBox2Checked==0)
                    {
                        if((tfc[i]=fopen(asfc.c_str(),"rt+"))==NULL)
                                {
                                        cout<<"  �޷�����ԭ������" <<(i+1)<<"������ϵ����Ա......"<<endl;
                                        return 0;
                                }
                    }
                //���ĸ���Ϊ�߲�����
                //if( check[2]==100 && i==4 )
                if(i==StreamMaxCyl)
                   {
                      tfp[RstreamNum]=fopen(asfp.c_str(),"rt+");
                      if(CheckBox2Checked==0)
                          tfc[RstreamNum]=fopen(asfc.c_str(),"rt+");
                      distribut[RstreamNum]=distribut[i];
                   }
                /*
                if( check[2]==0  )
                   {
                      tfp[RstreamNum]=fopen(asfp.c_str(),"rt+");;
                      if(CheckBox2Checked==0)
                          tfc[RstreamNum]=fopen(asfc.c_str(),"rt+");
                      distribut[RstreamNum]=distribut[i];
                      check[2]++;
                   }
                 */
        } //if(distribut[streams]>0)

     } //for(streams=1;streams< ;streams++)

   check[13]=0;
   inf_time=check[0]-check[10]/streamsnum;//��ʵ��Чʱ���
   checkinf_time=inf_time;


   //******�ϲ�streams****************

   int h[RstreamNum+1],m[RstreamNum+1],sec[RstreamNum+1],mes[RstreamNum+1],times[RstreamNum+1],timest[RstreamNum+1];   //������ʱ���ԭ��ʱ��һ��

  do
    {
       //*************�ϲ�����5����***********
   double ** data ,** datat,** datar, **TempResult ;        // ������STEP 1: SET UP THE ROWS.
   data = new double*[RstreamNum+1] ;
   datat = new double*[RstreamNum+1] ;
   datar = new double*[RstreamNum+1] ;
   TempResult= new double*[RstreamNum+1] ;

   for (j = 0; j < RstreamNum+1; j++)    //new ����������Ϊ����
        {
                data[j] = new double[1000];
                datat[j] = new double[1000];
                datar[j] = new double[1000];
                TempResult[j] = new double[1000];
        }
    //*************�ϲ�ԭ��,ԭ���������ʱ����һ�µ�*****************
     if( check[13]!=0 )
        {
                check[6]=0;
                checkinf_time=inf_time; //�ϲ�ԭ��ʱ��checkinf_time check[6]�ص����ֵ
                if(CheckBox2Checked==0)
                   {
                        for( i=0;i<streamsnum;i++ )
                                tfp[filenum[i]]=tfc[filenum[i]];
                        tfp[RstreamNum]=tfc[RstreamNum];
                   }
        }
     check[13]++; //�ж�ֻѭ��2��
     //*************

      NextTime:   //*****************************ʱ��νڵ�********************

      if(check[6]==0 && (inf_time+1-checkinf_time)!=1)
                printf("  ��%d��ʱ�����ȡ�ĸ���������ʱ���޷�ƥ��,��Чʱ���̫��,���޸�ʱ��λ������!\n\r",inf_time-checkinf_time);
      check[6]=0;
      if(check[13]==1 )
                cout<<"  ���ںϲ���" <<(inf_time+1-checkinf_time)<< "��ʱ�������������,���Ժ�......"<<endl;
      if(check[13]==2 )
         if(CheckBox2Checked==0)
                cout<<"  ���ںϲ���" <<(inf_time+1-checkinf_time)<< "��ʱ���ԭ������,���Ժ�......"<<endl;
         else
                break;

      //---------------ʮ�������----��ʱ���������ʱ��ν���ļ���ԭ�벻�ֿ���-------------------------------
      if( check[13]==1 ) //***������***
        {
                if( inf_time-checkinf_time!=0 )
                        {

                                fwrite(&TimeId,4,1,TReng);
                                fwrite(&TimeId,4,1,TReng);
                                fwrite(&TimeId,4,1,TReng);
                                fwrite(&TimeId,4,1,TReng);
                        }
                fwrite(&infs.BegH[inf_time-checkinf_time],4,1,TReng);
                fwrite(&infs.BegM[inf_time-checkinf_time],4,1,TReng);
                fwrite(&infs.BegS[inf_time-checkinf_time],4,1,TReng);
                fwrite(&infs.EndH[inf_time-checkinf_time],4,1,TReng);
                fwrite(&infs.EndM[inf_time-checkinf_time],4,1,TReng);
                fwrite(&infs.EndS[inf_time-checkinf_time],4,1,TReng);
                fwrite(&infs.Rate[inf_time-checkinf_time],4,1,TReng);
        }
      if( check[13]==2 ) //***ԭ��***
        {
                if( inf_time-checkinf_time!=0 )
                        {
                                fwrite(&TimeId,4,1,TRcod);
                                fwrite(&TimeId,4,1,TRcod);
                                fwrite(&TimeId,4,1,TRcod);
                                fwrite(&TimeId,4,1,TRcod);
                        }
                fwrite(&infs.BegH[inf_time-checkinf_time],4,1,TRcod);
                fwrite(&infs.BegM[inf_time-checkinf_time],4,1,TRcod);
                fwrite(&infs.BegS[inf_time-checkinf_time],4,1,TRcod);
                fwrite(&infs.EndH[inf_time-checkinf_time],4,1,TRcod);
                fwrite(&infs.EndM[inf_time-checkinf_time],4,1,TRcod);
                fwrite(&infs.EndS[inf_time-checkinf_time],4,1,TRcod);
                fwrite(&infs.Rate[inf_time-checkinf_time],4,1,TRcod);
        }
      //---------------ʮ�������----��ʱ���������ʱ��ν���ļ���ԭ�벻�ֿ���-------------------------------
      //************t1 t2 t3��ֵ��0***************
      for (i = 0; i < RstreamNum+1; i++)
        {
                times[i]=0;
                timest[i]=0;

        }
      //**************��ȡÿ���� T1***************
      for(i=0;i<streamsnum;i++) //��˳�����,��˳��RstreamNum������
        {
                //*********************������**************************************
                fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                times[filenum[i]]= (h[filenum[i]]*3600+m[filenum[i]]*60+sec[filenum[i]])*1000+mes[filenum[i]];
                for(j=0;j<distribut[filenum[i]];j++)  //distribut[filenum[i]] ������
                        fscanf(tfp[filenum[i]],"%lf",&data[filenum[i]][j]);// filenum[i] ����
        }

     //**************��ȡ T3,ʹ��t3>=t1***************
     for(i=0;i<streamsnum;i++)
       while(times[RstreamNum]<times[filenum[i]])    //t3<t1
        {
                if( feof(tfp[RstreamNum])  && checkinf_time==inf_time )   //���е�ʱ��ζ�������
                        {
                                cout<<"��"<<filenum[0]<<"��������ʱ��������,����ϵ����Ա......"<<endl;
                                return 0;
                        }
                if( feof(tfp[RstreamNum]) )  goto lab2;
                fscanf(tfp[RstreamNum],"%d:%d:%d:%d", &h[RstreamNum],&m[RstreamNum],&sec[RstreamNum],&mes[RstreamNum]);
                times[RstreamNum]= (h[RstreamNum]*3600+m[RstreamNum]*60+sec[RstreamNum])*1000+mes[RstreamNum];

                 //************************��(checkinf_time>1)��ʱ��εĵ�һ������(t3>t1)�Ĳ���ѡȡʱ���쳣��������****************************************
                if( h[RstreamNum]==TimeId && m[RstreamNum]==TimeId && sec[RstreamNum]==TimeId && mes[RstreamNum]==TimeId )//�ж��Ƿ���ʱ��εĽ�����־
                                {
                                        if(feof(tfp[RstreamNum]))  goto lab2;
                                        //**************����������ͳһ��ͬһʱ��εĽ�����־��**********
                                        for(i=0;i<streamsnum;i++)
                                                while(h[filenum[i]]!=TimeId || m[filenum[i]]!=TimeId || sec[filenum[i]]!=TimeId || mes[filenum[i]]!=TimeId)
                                                        {
                                                                if(feof(tfp[filenum[i]]))  goto lab2;
                                                                fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                                                                if(h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                                                        break;
                                                                for(j=0;j<distribut[filenum[i]];j++)
                                                                        {
                                                                                fscanf(tfp[filenum[i]],"%lf",&data[filenum[i]][j]);
                                                                                //fscanf(tfc[filenum[i]],"%lf",&Codata[filenum[i]][j]); //***ԭ��***
                                                                        }
                                                        }
                                        checkinf_time=checkinf_time-1;
                                        if(checkinf_time!=0) goto  NextTime;
                                } 
                for(j=0;j<distribut[RstreamNum];j++)
                                fscanf(tfp[RstreamNum],"%lf",&data[RstreamNum][j]);
          }
      //**************��ȡÿ���� T2,ʹ��t2>t3***************
      for(i=0;i<streamsnum;i++)
        while(times[RstreamNum]>timest[filenum[i]]) //t3>t2
                {
                        if( feof(tfp[filenum[i]]) && checkinf_time==inf_time )
                                {
                                        cout<<"��"<<filenum[i]<<"��������ʱ��������,����ϵ����Ա......"<<endl;
                                        return 0;
                                }
                        if( feof(tfp[filenum[i]]) )  goto lab2;
                        fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                        timest[filenum[i]]= (h[filenum[i]]*3600+m[filenum[i]]*60+sec[filenum[i]])*1000+mes[filenum[i]];

                        //************************��(checkinf_time>1)��ʱ��εĵ�һ������(t3<t2)�Ĳ���ѡȡʱ���쳣��������****************************************
                        if( h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                {
                                        if(feof(tfp[filenum[i]]))    goto lab2;
                                        //**************����������ͳһ��ͬһʱ��εĽ�����־��**********
                                        for(i=0;i<streamsnum;i++)
                                        while(h[filenum[i]]!=TimeId || m[filenum[i]]!=TimeId || sec[filenum[i]]!=TimeId || mes[filenum[i]]!=TimeId)
                                                {
                                                        if(feof(tfp[filenum[i]]))  goto lab2;   //������ֵ����
                                                        fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                                                        if(h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                                                break;
                                                        for(j=0;j<distribut[filenum[i]];j++)
                                                                fscanf(tfp[filenum[i]],"%lf",&data[filenum[i]][j]);
                                                }
                                         
                                        while(h[RstreamNum]!=TimeId && m[RstreamNum]!=TimeId && sec[RstreamNum]!=TimeId && mes[RstreamNum]!=TimeId )
                                                {
                                                        if(feof(tfp[RstreamNum]))   goto lab2;
                                                        fscanf(tfp[RstreamNum],"%d:%d:%d:%d", &h[RstreamNum],&m[RstreamNum],&sec[RstreamNum],&mes[RstreamNum]);
                                                        if(h[RstreamNum]==TimeId && m[RstreamNum]==TimeId && sec[RstreamNum]==TimeId && mes[RstreamNum]==TimeId )
                                                            break;
                                                        for(j=0;j<distribut[RstreamNum];j++)
                                                                fscanf(tfp[RstreamNum],"%lf",&data[RstreamNum][j]);
                                                }
                                        checkinf_time=checkinf_time-1;
                                        if(checkinf_time!=0) goto  NextTime;    //��ʼ��һ��ʱ��εĲ�ֵ����
                                }
                          //************************
                          
                        for(j=0;j<distribut[filenum[i]];j++)
                                fscanf(tfp[filenum[i]],"%lf",&datat[filenum[i]][j]);

                        //������������㣬����n1/t1 ==  n2/t2
                        if(times[RstreamNum]>timest[filenum[i]])
                            {
                                //***************����t1=t2********************
                                times[filenum[i]]=timest[filenum[i]];
                                for(j=0;j<distribut[filenum[i]];j++)
                                        data[filenum[i]][j]=datat[filenum[i]][j];
                            }
                }
      /*
            data[filenum[i]][j]    -> n1
            data[filenum[0]][j]    -> n3
            datat[filenum[i]][j]   -> n2

            times[filenum[i]]     ->t1
            times[filenum[0]]     ->t3
            timest[filenum[i]]    ->t2

            n3[i]=(n2[i]-n1[i])*t3/(t2-t1)+(n1[i]*t2-n2[i]*t1)/(t2-t1);

        */

     if(stop) return 0;    


     while(1)
      {
        summation=0;

   //***************��ÿ������ÿ���������в�ֵ����*******************************
         for(i=0;i<streamsnum;i++)
             {
                if( (times[RstreamNum]<=timest[filenum[i]]) && (times[RstreamNum]>=times[filenum[i]]) ) //if((t3<=t2)&&(t3>=t1))
                       {
                         //��ֵ.
                         //if(filenum[i]==4)
                         if(filenum[i]==StreamMaxCyl)
                            {
                                for(j=0;j<distribut[filenum[i]];j++)
                                   {
                                      result[ summation ]= data[RstreamNum][j];
                                      summation++;   //����
                                   }
                            }
                         else
                            {
                                 for(j=0;j<distribut[filenum[i]];j++)
                                    {
                                        //20110421���Բ�ֵ�޸�Ϊ�ƽ���ֵ
                                        //datar[filenum[i]][j]=(datat[filenum[i]][j] -data[filenum[i]][j])*times[RstreamNum]/(timest[filenum[i]]-times[filenum[i]])+(data[filenum[i]][j] *timest[filenum[i]]-datat[filenum[i]][j]*times[filenum[i]])/(timest[filenum[i]]-times[filenum[i]]);
                                        if( fabs(timest[filenum[i]]-times[RstreamNum]) <  fabs(times[RstreamNum]-times[filenum[i]]) )
                                              //t2-t3 < t3-t1 �ƽ�ȡn2
                                                datar[filenum[i]][j]=datat[filenum[i]][j];
                                        else//( fabs(timest[filenum[i]]-times[filenum[0]]) >  fabs(times[filenum[0]]-times[filenum[i]]) )
                                             //t2-t3 >=t3-t1 �ƽ�ȡn1
                                                datar[filenum[i]][j]=data[filenum[i]][j];

                                        //�߲�����ǰ�ķ�ʽ�ϲ�
                                        //����ȡֵ��1/2��ȡ���ظ��ƽ�ֵ��ȡ��DeiDa T�ıƽ���ʽ��õ�����ֵ
                                       if(fabs(timest[filenum[i]]-times[RstreamNum]) ==  fabs(times[RstreamNum]-times[filenum[i]])
                                          && heads.frdepth[StreamMaxCyl]*heads.frpersec[StreamMaxCyl]/(heads.frdepth[filenum[i]]*heads.frpersec[filenum[i]])<2  )

                                          {
                                           if(datar[filenum[i]][j]==TempResult[filenum[i]][j] )
                                                {
                                                    datar[filenum[i]][j]=datat[filenum[i]][j];
                                                }
                                           }
                                        result[ summation ]=  datar[filenum[i]][j];
                                        summation++;   //����
                                    }
                            }
                        }
              } //for(i=0;i<streamsnum;i++)
         if( (summation==inf_paranumber) && (times[RstreamNum]<=timest[filenum[1]]) && (times[RstreamNum]>=times[filenum[1]])  &&  (times[RstreamNum]<=timest[filenum[i-1]]) && (times[RstreamNum]>=times[filenum[i-1]])  )//if((t3<t2)&&(t3>=t1))???
                 {
                        for(i=0;i<streamsnum;i++)
                            {
                                if(filenum[i]==StreamMaxCyl)
                                        continue;
                                for(j=0;j<distribut[filenum[i]];j++)
                                   {
                                        //����������е�ֵ����¼�������Ա��Ƚ�
                                        TempResult[filenum[i]][j]= datar[filenum[i]][j];
                                   }
                             }   
                        //*********print out******************
                        if(p!=-1)
                           {
                                 check[6]++;
                                 if( check[13]==1 ) //***������***
                                        {
                                                fwrite(&h[RstreamNum],4,1,TReng);
                                                fwrite(&m[RstreamNum],4,1,TReng);
                                                fwrite(&sec[RstreamNum],4,1,TReng);
                                                mestime=(float)mes[RstreamNum];
                                                fwrite(&mestime,4,1,TReng);
                                                for(j=0;j<summation;j++)
                                                   {
                                                        fwrite(&result[infs.order[j]],sizeof(result[infs.order[j]]),1,TReng);
                                                      //  fprintf(Test,"   %12.6f",result[infs.order[j]]);
                                                   }
                                               // fprintf(Test,"\n");
                                        }
                                 if( check[13]==2 )  //***ԭ��***
                                        if(CheckBox2Checked==0)
                                                {
                                                        fwrite(&h[RstreamNum],4,1,TRcod);
                                                        fwrite(&m[RstreamNum],4,1,TRcod);
                                                        fwrite(&sec[RstreamNum],4,1,TRcod);
                                                        mestime=(float)mes[RstreamNum];
                                                        fwrite(&mestime,4,1,TRcod);
                                                        for(j=0;j<summation;j++)
                                                                {
                                                                        Nresult[infs.order[j]]=( int )floor(result[infs.order[j]]);
                                                                        fwrite(&Nresult[infs.order[j]],sizeof(Nresult[infs.order[j]]),1,TRcod);
                                                                }
                                                }
                           }
                        //********************read next t3**************************************
                        if(feof(tfp[RstreamNum]))  goto lab2;
                        p=fscanf(tfp[RstreamNum],"%d:%d:%d:%d", &h[RstreamNum],&m[RstreamNum],&sec[RstreamNum],&mes[RstreamNum]);
                        times[RstreamNum]= (h[RstreamNum]*3600+m[RstreamNum]*60+sec[RstreamNum])*1000+mes[RstreamNum];

                        if(h[RstreamNum]==TimeId && m[RstreamNum]==TimeId && sec[RstreamNum]==TimeId && mes[RstreamNum]==TimeId )//�ж��Ƿ���ʱ��εĽ�����־
                                {

                                        if(feof(tfp[RstreamNum]))  goto lab2;

                                        //**************����������ͳһ��ͬһʱ��εĽ�����־��**********
                                        for(i=0;i<streamsnum;i++)
                                                while(h[filenum[i]]!=TimeId || m[filenum[i]]!=TimeId || sec[filenum[i]]!=TimeId || mes[filenum[i]]!=TimeId)
                                                        {
                                                                if(feof(tfp[filenum[i]]))  goto lab2;
                                                                fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                                                                if(h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                                                        break;
                                                                for(j=0;j<distribut[filenum[i]];j++)
                                                                        fscanf(tfp[filenum[i]],"%lf",&data[filenum[i]][j]);
                                                        }
                                        checkinf_time=checkinf_time-1;
                                        if(checkinf_time!=0) goto  NextTime;
                                }

                        for(j=0;j<distribut[RstreamNum];j++)
                                fscanf(tfp[RstreamNum],"%lf",&data[RstreamNum][j]);
                          

                        if(stop) return  0;
                        //***************����T3����   t3<t2  and t3<t1��ȡ��һ��T3********************
                        for(i=0;i<streamsnum;i++)
                            while( times[RstreamNum]<times[filenum[i]] ) //   t3<t1 ->  next t3  ��֤t3>=t1
                                {
                                        if(feof(tfp[RstreamNum]))   goto lab2;
                                        p=fscanf(tfp[RstreamNum],"%d:%d:%d:%d", &h[RstreamNum],&m[RstreamNum],&sec[RstreamNum],&mes[RstreamNum]);
                                        times[RstreamNum]= (h[RstreamNum]*3600+m[RstreamNum]*60+sec[RstreamNum])*1000+mes[RstreamNum];

                                        if(h[RstreamNum]==TimeId && m[RstreamNum]==TimeId && sec[RstreamNum]==TimeId && mes[RstreamNum]==TimeId )//�ж��Ƿ���ʱ��εĽ�����־
                                                {
                                                        if(feof(tfp[RstreamNum]))  goto lab2;

                                                        //**************����������ͳһ��ͬһʱ��εĽ�����־��**********
                                                        for(i=0;i<streamsnum;i++)
                                                                while(h[filenum[i]]!=TimeId || m[filenum[i]]!=TimeId || sec[filenum[i]]!=TimeId || mes[filenum[i]]!=TimeId)
                                                                        {
                                                                                if(feof(tfp[filenum[i]]))  goto lab2;
                                                                                fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                                                                                if(h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                                                                        break;
                                                                                for(j=0;j<distribut[filenum[i]];j++)
                                                                                        fscanf(tfp[filenum[i]],"%lf",&data[filenum[i]][j]);
                                                                        }
                                                        checkinf_time=checkinf_time-1;
                                                        if(checkinf_time!=0) goto  NextTime;
                                                }
                                                
                                       for(j=0;j<distribut[RstreamNum];j++)
                                                fscanf(tfp[RstreamNum],"%lf",&data[RstreamNum][j]);
                                }
                 } //if (summation==inf_paranumber)
         else    //must t3>=t1  t2>t1   ->t3<t2  t2>t1 
            {
              for(i=0;i<streamsnum;i++)
                {
                        //read next t2
                        while( times[RstreamNum]>timest[filenum[i]] )//t3>t2 ->next t2 ��֤t3<=t2
                             {
                                //***************����t1=t2********************
                                times[filenum[i]]=timest[filenum[i]];
                                for(j=0;j<distribut[filenum[i]];j++)
                                        data[filenum[i]][j]=datat[filenum[i]][j];

                                if(feof(tfp[filenum[i]]))    goto lab2;
                                fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                                timest[filenum[i]]= (h[filenum[i]]*3600+m[filenum[i]]*60+sec[filenum[i]])*1000+mes[filenum[i]];

                                if(h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                     {
                                        if(feof(tfp[filenum[i]]))    goto lab2;
                                        //**************����������ͳһ��ͬһʱ��εĽ�����־��**********
                                        for(i=0;i<streamsnum;i++)
                                            while(h[filenum[i]]!=TimeId || m[filenum[i]]!=TimeId || sec[filenum[i]]!=TimeId || mes[filenum[i]]!=TimeId)
                                                {
                                                        if(feof(tfp[filenum[i]]))  goto lab2;   //������ֵ����
                                                        fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                                                        if(h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                                                break;
                                                        for(j=0;j<distribut[filenum[i]];j++)
                                                                fscanf(tfp[filenum[i]],"%lf",&data[filenum[i]][j]);
                                                }
                                        while(h[RstreamNum]!=TimeId && m[RstreamNum]!=TimeId && sec[RstreamNum]!=TimeId && mes[RstreamNum]!=TimeId )
                                                {
                                                        if(feof(tfp[RstreamNum]))   goto lab2;
                                                        fscanf(tfp[RstreamNum],"%d:%d:%d:%d", &h[RstreamNum],&m[RstreamNum],&sec[RstreamNum],&mes[RstreamNum]);
                                                        if(h[RstreamNum]==TimeId && m[RstreamNum]==TimeId && sec[RstreamNum]==TimeId && mes[RstreamNum]==TimeId )
                                                            break;
                                                        for(j=0;j<distribut[RstreamNum];j++)
                                                                fscanf(tfp[RstreamNum],"%lf",&data[RstreamNum][j]);
                                                }
                                        checkinf_time=checkinf_time-1;
                                        if(checkinf_time!=0) goto  NextTime;    //��ʼ��һ��ʱ��εĲ�ֵ����
                                     }

                                for(j=0;j<distribut[filenum[i]];j++)
                                        fscanf(tfp[filenum[i]],"%lf",&datat[filenum[i]][j]);
                             }
                  }//for(i=0;i<streamsnum;i++)
           } // else
        } //while(1)

   lab2: 
   if(check[6]==0)
        printf("  ��%d��ʱ�����ȡ�ĸ���������ʱ���޷�ƥ��,��Чʱ���̫��,���޸�,���޸�ʱ��λ������!\n\r",inf_time-checkinf_time+1);
   check[6]=0;

   for (i = 0; i < RstreamNum+1;  i++)
        {
                delete[] data[i];                 // STEP 1: DELETE THE COLUMNS
                delete[] datat[i];
                delete[] datar[i];
                delete[] TempResult[i];
         }
   delete[] data;                        // STEP 2: DELETE THE ROWS
   delete[] datar;
   delete[] datat;
   delete[] TempResult;


   
   for(i=0;i<streamsnum;i++)
        {
              fclose(tfp[filenum[i]]);
              fclose(tfp[RstreamNum]);
              if( check[13]==2 ){
                   if(CheckBox2Checked==0)
                        {
                                fclose(tfc[filenum[i]]);
                                DeleteFile( ChangeFileExt(TempFileName,".cod_tmp"+IntToStr(filenum[i])) );   //***ԭ��***
                        }
                   }
              else
                        DeleteFile( ChangeFileExt(TempFileName,".eng_tmp"+IntToStr(filenum[i])) );   //ɾ��������м��ļ�
         }
  }while(check[13]==1);

  for(i=0;i<4;i++)  //���һ��ʱ���
        {
                fwrite(&TimeId,4,1,TReng);
                if(CheckBox2Checked==0)
                        fwrite(&TimeId,4,1,TRcod);
        }
   //clreol();
  cout<<"  ���ݺϲ�����������������ݣ����Ժ�......\n"<<endl;

  //fclose(Test);
  return 1;
  }
  catch( ... )
        {
                cout<<"  �������쳣������ϵ����Ա......"<<endl;
                return 0;
        }

 }
//------------------------------------------------------------------------------
 int read_infs()
 {
  try
  {
  if((inf=fopen(Dir.c_str(),"r"))==NULL)
             {
             cout<<"  ��Ϣ�ļ��޷���!"<<endl;
              Beep(100,100);
              return 0;
             }
  cout<<"\n  ���ڷ�����Ϣ�ļ�......"<<endl;
  fscanf(inf,"%s \n",infs.SYSINPUT);
  if(stricmp(infs.SYSINPUT,"@SYSINPUT@")!=0)
           return 0;
  fscanf(inf,"%s \n",infs.Planename);
  fscanf(inf,"%s \n",infs.Planeno);
  fscanf(inf,"%s \n",infs.FlightDate);
  fscanf(inf,"%s \n",infs.FlightNo);
  fscanf(inf,"%s \n",infs.MeasureSystem); //����ϵͳ���ַ���
  fscanf(inf,"%s \n",infs.DataProperty);
  fscanf(inf,"%lf \n",&infs.Weight);      //�ɻ����������ʵ�ͣ�
  fscanf(inf,"%f \n",&infs.Core);
  fscanf(inf,"%s \n",infs.Hang);
  fscanf(inf,"%f \n",&infs.Temperature);
  fscanf(inf,"%f \n",&infs.Press);       //��ѹ��ʵ�ͣ�
  fscanf(inf,"%f \n",&infs.Windspeed);
  fscanf(inf,"%f \n",&infs.View);
  fscanf(inf,"%s \n",infs.Ground);
  fscanf(inf,"%s \n",infs.Order);
  fscanf(inf,"%s \n",infs.Testperson);
  
  fscanf(inf,"%s \n",infs.FlightDataFile); //�����ļ�����(������·��) ���ַ���
  fscanf(inf,"%s \n",infs.DataHeadfile);    //У׼�ļ�����(������·��) ���ַ���

  fscanf(inf,"%s \n",infs.SubjectName);
  fscanf(inf,"%s \n",infs.UserID);          //���ݴ����û����ƣ��ַ���
  fscanf(inf,"%s \n",infs.ProcessDate);
  fscanf(inf,"%s \n",infs.SYSINPUTEND);     //�����֣�SYSINPUTEND���ض��������
  if(stricmp(infs.SYSINPUTEND,"@SYSINPUTEND@")!=0)
           return 0;

  fscanf(inf,"%s \n",infs.PARINFOR);      // @PARINFOR@   //�����֣�ParInfor�����������Ϣ
  if(stricmp(infs.PARINFOR,"@PARINFOR@")!=0){
           cout<<"  ��Ϣ�ļ����޴��������Ϣ......"<<endl;
           return 0;
           }
  i=0;
  fscanf(inf,"%s \n",infs.Par_name[i]);     //������(�ַ�)
  fscanf(inf,"%s \n",infs.Par_unit[i]);     //��λ(�ַ�)
  fscanf(inf,"%lf \n",&infs.Par_UP[i]);
  fscanf(inf,"%lf \n",&infs.Par_down[i]);
  fscanf(inf,"%s \n",infs.Par_Group[i]);
  fscanf(inf,"%s \n",infs.Par_note[i]);     //������ע���ַ���
  fscanf(inf,"%s \n",infs.PARINFOREND);  //@PARINFOREND@  //�����֣�ParINforEND��������Ϣ����
  while(stricmp(infs.PARINFOREND,"@PARINFOREND@")!=0)
        {
                i++;
                strcpy(infs.Par_name[i],infs.PARINFOREND);
                if(strlen(infs.Par_name[i])<3)
                {
                   cout<<"  ���ݴ����б������ϢINF�ļ����ɴ����޷�����"<<endl;
                   return  0;
                }
                fscanf(inf,"%s \n",infs.Par_unit[i]);     //��λ(�ַ�)
                fscanf(inf,"%lf \n",&infs.Par_UP[i]);     //���ޣ�ʵ�ͣ�
                fscanf(inf,"%lf \n",&infs.Par_down[i]);   //���ޣ�ʵ�ͣ�
                fscanf(inf,"%s \n",infs.Par_Group[i]);    //������ͨ�����ַ���
                fscanf(inf,"%s \n",infs.Par_note[i]);     //������ע���ַ���
                fscanf(inf,"%s \n",infs.PARINFOREND);
        }
        
  /*****************�Բ�����Ϣ���й��ɴ���*******************************************/
  inf_paranumber=i+1; //ѡȡ�Ĳ�������
  k=0;
  for(j=0;j<RstreamNum;j++)
        {
                filenum[j]=0;
                for(i=0;i<inf_paranumber;i++)
                        {
                                inf_stream=StrToInt( infs.Par_name[i][strlen(infs.Par_name[i])-1] );
                                if(inf_stream>RstreamNum || inf_stream==0)
                                        {
                                                cout<<"  ��ȡ�ò������������������飡"<<endl;
                                                return  0;
                                        }
                                //************�ظ����������ʾ******************
                                for( int jj=0;jj<i;jj++ )
                                        {
                                                if( stricmp( infs.Par_name[i],infs.Par_name[jj])==0 )
                                                        {
                                                                cout<<"  "<<infs.Par_name[i]<<"�ظ�ѡȡ�����޸ġ�"<<endl;
                                                                return  0;
                                                        }
                                        }
                                //***********��¼����λ��*********************
                                if( inf_stream==(j+1) )
                                        {
                                                strcpy(infs.Par_order[k] ,infs.Par_name[i]); //��ÿ�����Ĳ�������������
                                                k++;
                                                distributary[j][distribut[j]]=i;
                                                distribut[j]++;    //������������
                                                if(distribut[j]>1000)
                                                        {
                                                                cout<<" ����������̫��,����1000��,���ܴ���..."<<endl;
                                                                return 0;
                                                        }
                                        }
                          }
          }
  //************�����账����������Լ��������****************
   streamsnum=0;
   for(i=0;i<RstreamNum;i++)
        {
                if(distribut[i]>0)
                        {
                                filenum[streamsnum]=i;  //�账��ľ�������
                                streamsnum++;   //�账��������
                        }
        }
   if( streamsnum<1 )  return 0;
   //************
   if(streamsnum==1)
        Singlestr=1; //��������

   //----------------------infs.order[j]��¼ÿ���������м�infs.Par_order��λ��---
   j=0;
   for( i=0;i<inf_paranumber;i++ )
        {
	        for( k=0;k<inf_paranumber;k++ )
                        {
                                if(stricmp(infs.Par_order[k],infs.Par_name[i])==0)
                                        {
                                                infs.order[j]=k;
                                                j++;
                                        }
                        }
        }
  //----------------------------------------------------------------------------
  /*****************************************************************************/
  fscanf(inf,"%s \n",infs.TIMEINFO);
  if(stricmp(infs.TIMEINFO,"@TIMEINFO@")!=0)
        {
                cout<<"  ��Ϣ�ļ�����ʱ���......"<<endl;
                return 0;
        }
  i=0;
  fscanf(inf,"%d \n",&infs.BegH[i]);
  fscanf(inf,"%d \n",&infs.BegM[i]);
  fscanf(inf,"%d \n",&infs.BegS[i]);
  fscanf(inf,"%d \n",&infs.EndH[i]);
  fscanf(inf,"%d \n",&infs.EndM[i]);
  fscanf(inf,"%d \n",&infs.EndS[i]);
  fscanf(inf,"%d \n",&infs.Rate[i]);

  if(infs.BegH[i]>24 || infs.BegM[i]>60)
        {
                cout<<"  ʱ���ʱ����Ϣ������Χ������ȷ��......"<<endl;
                return 0;
        }
  if(infs.EndH[i]>24) infs.EndH[i]=24;
  if(infs.EndM[i]>60) infs.EndM[i]=60;
  infs.BegTime[i]=3600*infs.BegH[i]+ 60*infs.BegM[i]+ infs.BegS[i];
  infs.EndTime[i]=3600*infs.EndH[i]+ 60*infs.EndM[i]+ infs.EndS[i];

  fscanf(inf,"%s \n",infs.TIMEINFOREND);
  while(stricmp(infs.TIMEINFOREND,"@TIMEINFOREND@")!=0)
        {
                if(feof(inf))
                {
                        inf_time=1;
                        break;
                }
                i++;
                infs.BegH[i]=StrToInt(infs.TIMEINFOREND);
                fscanf(inf,"%d \n",&infs.BegM[i]);
                fscanf(inf,"%d \n",&infs.BegS[i]);
                fscanf(inf,"%d \n",&infs.EndH[i]);
                fscanf(inf,"%d \n",&infs.EndM[i]);
                fscanf(inf,"%d \n",&infs.EndS[i]);
                fscanf(inf,"%d \n",&infs.Rate[i]);
                fscanf(inf,"%s \n",infs.TIMEINFOREND);
                if(infs.BegH[i]>24 || infs.BegM[i]>60 || i>99 )
                        {
                                cout<<"  ʱ��δ������ʱ��γ���100����ȷ��......"<<endl;
                                return 0;
                        }
                if(infs.EndH[i]>24) infs.EndH[i]=24;
                if(infs.EndM[i]>60) infs.EndM[i]=60;
                infs.BegTime[i]=3600*infs.BegH[i]+ 60*infs.BegM[i]+ infs.BegS[i];
                infs.EndTime[i]=3600*infs.EndH[i]+ 60*infs.EndM[i]+ infs.EndS[i];
     }
  inf_time=i+1;//һ����ʱ���
  check[0]= inf_time;
  fscanf(inf,"%s \n",infs.SYSOUTPUT); //@SYSOUTPUT@                  //�����֣�SYSOUTPUT��ϵͳ�ض������Ϣ
  fscanf(inf,"%s \n",infs.FilePath);
  fscanf(inf,"%d\n",&CheckBox2Checked);  //�Ƿ����Դ�� -1 �� 0 �ǡ�
  fscanf(inf,"%s \n",infs.SYSOUTPUTEND);

  fscanf(inf,"%s \n",infs.INPUT);
  fscanf(inf,"%s \n",infs.INPUTEND);
  fscanf(inf,"%s \n",infs.OUTPUT);
  fscanf(inf,"%s \n",infs.OUTPUTEND);
 /* if(strcmp( "@OUTPUTEND@",infs.OUTPUTEND)!=0)
  {
     cout<<"  ���ݴ����б������ϢINF�ļ����ɴ����޷�����"<<endl;
     return  0;
  }*/
  fclose(inf);
  return 1;

  }
 catch ( ... )
        {
                cout<<"  ��Ϣ�ļ��쳣,����ϵ����Ա......"<<endl;
                exit(1);
        }
 return 1;
 }
//-----------------------------------------------------------------------------
int read_hea()
 {
  try{
    char nouse[100];
    int i=0,j=0;

    double heaTempDouble;  //�¼���Hea_Temp
    char heaTempChar[50];  //�¼���Hea_Temp
    int heaTemp,start[50];
    double efficient[50];

    all_count=0;
    head_fname=infs.DataHeadfile;
    if((hp=fopen(head_fname.c_str(),"r"))==NULL)
        {
              cout<<"  ��ͷ�ļ��޷���!"<<endl;
              Beep(100,100);
              return 0;
        }
    cout<<"\n  ���ڼ���У���ļ�......"<<endl;

//�¼���ͷ��Ϣ
            fscanf(hp,"%s \n",nouse);
            fscanf(hp,"%s \n",nouse);
            fscanf(hp,"%s %s\n",nouse,heaTempChar);
            fscanf(hp,"%s %s\n",nouse,heaTempChar);

            AnsiString ss= heaTempChar;
            ss=UpperCase(ss.SubString(1,3));  //�ж��Ƿ�Ϊarj21�ɻ�

            fscanf(hp,"%s %d\n",nouse,&heads.stream);  //����
            if( heads.stream>RstreamNum )
                {
                        cout<<"  ���������ܴ���"<<RstreamNum<<"������ϵ����Ա......"<<endl;
                        return 0;
                }
            fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);
             MaxSingleParaNmu=0;
            for(j=0;j<heads.stream;j++){
                    for(i=0;i<6;i++)   fscanf(hp,"%s \n",nouse);  //The format description of stream 1:
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %d\n",nouse,&heads.parameter_number[j]);   //��������
                    if(MaxSingleParaNmu<heads.parameter_number[j])
                          MaxSingleParaNmu=heads.parameter_number[j];
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);   //λ����
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);
                    fscanf(hp,"%s %d\n",nouse,&heads.id_len[j]);//PCM�ֳ�
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    if(stricmp(strlwr(heaTempChar),"kam500")==0)
                          heads.time_mode[j]=4;
                    else if(stricmp(heaTempChar,"770")==0)
                          heads.time_mode[j]=5;
                    else if(stricmp(heaTempChar,"�ش�")==0)
                          heads.time_mode[j]=3;
                    else if(stricmp(strlwr(heaTempChar),"uma2000")==0)
                          heads.time_mode[j]=7;
                    else if(strcmp(strlwr(heaTempChar),"jqd")==0)
                          heads.time_mode[j]=6;
                    else
                          heads.time_mode[j]=0;   //if(stricmp(heaTempChar,"dm6")==0)
                    fscanf(hp,"%s %d\n",nouse,&heads.frlength[j]);  //��������
                    fscanf(hp,"%s %d\n",nouse,&heads.frdepth[j]);  //������/����
                    fscanf(hp,"%s %d\n",nouse,&heads.frpersec[j]);// ������/��
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);  //ͬ���ֳ���
                    fscanf(hp,"%s %s\n",nouse,heaTempChar); //ͬ����ƫ��
                    fscanf(hp,"%s %d\n",nouse,&heads.idnum[j]); // id word address
                    fscanf(hp,"%s %d\n",nouse,&heads.id_sbit[j]); // id word start bit
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);//ʱ�����
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);   //out�źż���
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);   //����λ
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);   //in�źż���
                    fscanf(hp,"%s :%s\n",nouse,heaTempChar);    //λ˳��
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);   //ͬ������
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble); //ͬ����λ��
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);      //���巽ʽ
                    fscanf(hp,"%s %s %s\n",nouse,heaTempChar,heaTempChar);  //ID��������
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);    //ID��ʼֵ
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);  //ѭ��ͬ��ֵ  0
                    }
     para=new struct parastruct[MaxSingleParaNmu];
     j=0;
     while(!feof(hp))
     {
        if(j==heads.stream) break;
        for(j=0;j<heads.stream;j++)
         {
            fscanf(hp,"%s %s\n",nouse,heaTempChar);  //the_parameter_description_of_stream 1:
            for(i=0;i<heads.parameter_number[j];i++)
               {
                 fscanf(hp,"%s  ", nouse);
                 if(stricmp(nouse,"calculate_parameter_description:")==0) break;
                 while(stricmp(nouse,"name:")!=0){
                        if(feof(hp))
                                break;
                        fscanf(hp,"%s  ", nouse);
                        }
                 if(feof(hp))
                        break;
                 fscanf(hp,"%s\n",para[i].name[j]);  //parameter name

                 while(stricmp(nouse,"cal_type:")!=0)
                               fscanf(hp,"%s ",nouse);
                 fscanf(hp,"%s\n",para[i].kind[j]);//У׼����  poly(����ʽ)  hyper(˫����) segm(���) no(��) ������У׼(bool)  �ⲿ����У׼(userfun)
                 fscanf(hp,"%s %s\n",nouse,para[i].code_type[j]); //ԭ������  bcd uint(�޷���)  sint���з��ţ� fcsf(16/32�ɿظ���)
                 fscanf(hp,"%s %s\n",nouse,heaTempChar);
                 while(stricmp(nouse,"rate:")!=0)
                               fscanf(hp,"%s ",nouse);
                 fscanf(hp," %d",&para[i].cyl[j]);// extrate value per sub f
                 fscanf(hp,"%s %d",nouse,&para[i].word_interval[j]); //*�¼�* �ּ��
                 fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);//����λ
                 fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);//�����Ƿ�ת
                 while(stricmp(nouse,"pcm_words:")!=0)
                               fscanf(hp,"%s ",nouse);
                 fscanf(hp," %d\n",&para[i].wordl[j]); //parameter name

                 fscanf(hp,"%s %d,%d,%d,%d,%d,%d\n",nouse,&para[i].fraddr[j],&para[i].fraddr1[j],&para[i].fraddr2[j],&para[i].fraddr3[j],&heaTempDouble,&heaTempDouble); //frame address
                 fscanf(hp,"%s %d,%d,%d,%d,%d,%d\n",nouse,&para[i].wdaddr[j],&para[i].wdaddr1[j],&para[i].wdaddr2[j],&para[i].wdaddr3[j],&heaTempDouble,&heaTempDouble); //word address
                 fscanf(hp,"%s ",nouse);
                 for(heaTemp=0;heaTemp<6;heaTemp++){
                          fscanf(hp,"%d, ",&start[heaTemp]);
                          para[i].bit_start[j][heaTemp]=start[heaTemp];    //  ȡλʱ����ʼλ��
                          }
                 fscanf(hp,"%s ",nouse);
                 for(heaTemp=0;heaTemp<6;heaTemp++){
                          fscanf(hp,"%d, ",&start[heaTemp]);
                          para[i].bit_len[j][heaTemp]=start[heaTemp];      //  ȡλʱ�ĳ��ȣ�
                          }
                 fscanf(hp,"%s ",nouse);
                 for(heaTemp=0;heaTemp<6;heaTemp++){
                          fscanf(hp,"%d, ",&start[heaTemp]);
                          if(ss=="ARJ")
                                start[heaTemp]=0;
                          para[i].bit_object[j][heaTemp]=start[heaTemp];      //  ȡλʱ��Ŀ��λ��
                          }                        
                 //fscanf(hp,"%s %lf,%lf,%lf,%lf,%lf,%lf\n",nouse,&heaTempDouble,&heaTempDouble,&heaTempDouble,&heaTempDouble,&heaTempDouble,&heaTempDouble);
                 fscanf(hp,"%s ",nouse);


                 if(stricmp(nouse,"coefficient:")==0){       //poly(����ʽ)
                         for(heaTemp=0;heaTemp<6;heaTemp++){
                             fscanf(hp,"%lf, ",&efficient[heaTemp]);
                             para[i].coefficient[j][heaTemp]= efficient[heaTemp];
                             }
                         }
                 else if(stricmp(nouse,"segment_number:")==0){    //segm(���)
                         fscanf(hp,"%d ",&para[i].lutn[j]);
                         for(heaTemp=0;heaTemp<para[i].lutn[j];heaTemp++) {
                              fscanf(hp,"%s %d,%lf",nouse,&start[heaTemp],&efficient[heaTemp]);
                              para[i].cod[j][heaTemp]=start[heaTemp];
                              para[i].phy[j][heaTemp]=efficient[heaTemp];
                             }
                         }

                 else if(stricmp(nouse,"segment_point:")==0){    //hyper(˫����)
                         fscanf(hp,"%lf",&para[i].segment_point[j]);
                         fscanf(hp,"%s ",nouse);
                         for(heaTemp=0;heaTemp<4;heaTemp++){
                             fscanf(hp,"%lf, ",&efficient[heaTemp]);
                             para[i].segment[j][heaTemp]= efficient[heaTemp];
                             }
                         fscanf(hp,"%s ",nouse);
                         for(heaTemp=4;heaTemp<8;heaTemp++){
                             fscanf(hp,"%lf, ",&efficient[heaTemp]);
                             para[i].segment[j][heaTemp]= efficient[heaTemp];
                             }
                         }  
                 else if(stricmp(nouse,"Description:")==0){
                        continue;
                        }
                 else
                      cout<<"  Ok!!"<<endl;
               } //for(i=1;i<parameter_number[j]+1;i++)

               par_count[j]=i;         //ÿ���������ݸ���
               all_count = all_count+par_count[j];   //�ܵ����ݸ���
         }// for(j=1;j<stream+1;j++)
     } // while(!feof(hp)) 
     fclose(hp);
     return 1;  
 }
 catch ( ... )
        {
                cout<<"  У���ļ��쳣,����ϵ����Ա......"<<endl;
                exit(1);
        }
 return 1;
}
//-----------------------------------------------------------------------------
int read_subf(int strea)  // read one whole subframe into subcod[8192];
 {
 try{
  int i,j;
  int idv,idadd,nwfr;
  unsigned int maskw;

  int DatErro=0;

  Word frame[2048];  //WORD   A 16-bit unsigned integer
  j=0;
  nwfr=heads.frlength[strea];    //һ�����ܵĶ�������
  idadd=heads.idnum[strea];
  maskw=0;
  for(i=0;i<heads.id_len[strea];i++)
        maskw=maskw+(short)pow(2,i);
  do
   {
        DatErro=DatErro+1;
        fread(&frame,2,nwfr,dp); ////һ�����ܣ�16-bit unsigned integer Ϊ2�ֽ�
        if(DatErro>heads.frdepth[strea]*10)
                {
                        cout<<"  ��ʾ�����ݳ���������ʧ10��(���������ʾ���������ݺ�У���ļ���ƥ��)......."<<endl;
                        DatErro=0;
                }
        if(heads.time_mode[strea]==6)
          idv=((frame[idadd]&0xFF)>>heads.id_sbit[strea])&maskw;
        else
          idv=(frame[idadd]>>heads.id_sbit[strea])&maskw;
        if((idv-j)!=0)
                {
                        j=0;
                        continue;
                }
        for(i=0;i<nwfr;i++)
                subcod[nwfr*j+i]=(frame[i]&0xffff); //һ������
                     // subcod[nwfr*j+i]=subcod[frlength*frame_no+word_no]
        j++;
   }while((j<heads.frdepth[strea])&&(!feof(dp)));
   if(feof(dp))
        return 0;
   return 1;
  }
 catch ( ... )
 {
  cout<<"  ���������쳣,����ϵ����Ա......"<<endl;
  exit(1);
 }
 return 1;
 }

//--------------------------------------------------------------------------------
double lut( __int64  c,int numb,int stre)  //lut subroutine.c: code; numb: order of parameter.  lut(vcod[i],i,streams)
 {
  int k,cc[34],pn,i;
  double  codd;
  double p=0.0,pp[34];
  long sigc,sigw;
  char kn[20];
  pn=extpar[numb].lutn[stre];
  strcpy(kn,extpar[numb].kind[stre]);
  //kn=extpar[numb].kind;
  for(k=0;k<34;k++)
   {
    cc[k]=0; pp[k]=0.0;
   }   

  //if(c>32768 && heads.time_mode[stre]==7)   //���uma2000�ɼ���У׼
  //     c=c-65535;

  codd=c;


  if( stricmp(extpar[numb].code_type[stre],"fcs618")==0 ) //fcs 618����У׼��ʽ 20121008
  {
     if( codd >=32768) codd=codd-65536;
     codd = 10*codd/32768 ;  
  }

try{
  for(k=0;k<extpar[numb].lutn[stre];k++)     //pn para[i].lutn[j]
   {
    cc[k]=extpar[numb].cod[stre][k];
    pp[k]=extpar[numb].phy[stre][k];
   }
}catch(...)
{
   int peng;
   peng++;
}


  if(stricmp(extpar[numb].code_type[stre],"sint")==0)   //�з��Ų�����-(~para+1)         ??�з��Ų�������
   {
    sigw=0;
    for(k=0;k<extpar[numb].wordl[stre];k++)
         sigw=sigw+extpar[numb].bit_len[stre][k];
    sigc=us_sig1(sigw,codd);
    codd=sigc;
   }
  //*******************************xint�����λΪ����λ��1Ϊ��ֵ��0Ϊ��ֵ*****************
   if(strcmp(extpar[numb].code_type[stre],"xint")==0)   //�з��Ų�����+-(para)         ??�з��Ų�������
  {   
        sigw=0;
        for(k=0;k<extpar[numb].wordl[stre];k++)
                   sigw=sigw+extpar[numb].bit_len[stre][k];
       sigc=us_sigx(sigw,codd);
       codd=sigc;
  }


  if(stricmp(kn,"poly")==0)//if(kn==2) //��������Ϊ��a0+x(a1+x(a2+x(a3+x(a4+x(a5+a6x)))))  extpar[numb].coefficient[j][heaTemp]
  {

    p=( extpar[numb].coefficient[stre][0]+codd*(extpar[numb].coefficient[stre][1]+codd*(extpar[numb].coefficient[stre][2]
        +codd*(extpar[numb].coefficient[stre][3]+codd*(extpar[numb].coefficient[stre][4]+codd*extpar[numb].coefficient[stre][5]))))  );
   return p;
  }
 if(stricmp(kn,"hyper")==0)//if(kn==3) //��������Ϊ��a/x+b; a/(b+cx)+d
  {

   if(codd>=extpar[numb].segment_point[stre]){
      try
        {
        p=extpar[numb].segment[stre][0]/(extpar[numb].segment[stre][1]+codd*extpar[numb].segment[stre][2])+extpar[numb].segment[stre][3];
        }catch(...){
                   p=codd;
                   }
         }
   else  {      
      try
        {
        p=extpar[numb].segment[stre][4]/(extpar[numb].segment[stre][5]+codd*extpar[numb].segment[stre][6])+extpar[numb].segment[stre][7];
        }catch(...){
                   p=codd;
                   }
        }
   return p;
  }


 //�����ֵ�ǲ�������
 //if(kn==4)  //��ֵ�и�ֵ��
 if(stricmp(kn,"fillcode")==0 )
 {
   sigw=0;
   for(k=0;k<extpar[numb].wordl[stre];k++)
        sigw=sigw+extpar[numb].bit_len[stre][k];
   sigc=us_sig(sigw,codd);
   if(sigc<cc[0]) {
        try{
                p=pp[0]-(cc[0]-sigc)*(pp[1]-pp[0])/(cc[1]-cc[0]);
           }catch(...){
                        p=sigc;
                    }
        return p;
        }
   for(k=1;k<pn;k++)
   if(sigc<cc[k]){
        try{
                p=pp[k-1]+(pp[k]-pp[k-1])*(sigc-cc[k-1])/(cc[k]-cc[k-1]);
           }catch(...){
                        p=sigc;
                      }
        return p;
        }
   try{
        p=pp[pn-1]+(pp[pn-1]-pp[pn-2])*(sigc-cc[pn-1])/(cc[pn-1]-cc[pn-2]);
       }catch(...){
                        p=sigc;
                  }
   return p;
 }  //end of kn==4;    

//���У׼  segm
 if(stricmp(kn,"segm")==0) {
  if(codd<cc[0])
  {
    try
        {
          p=pp[0]-(cc[0]-codd)*(pp[1]-pp[0])/(cc[1]-cc[0]);
        }catch(...){
        p=codd;
        }
//   if(p>99999999.999) p=99999999.999;
   return p;
  }

  for(k=1;k<extpar[numb].lutn[stre];k++)
   if(codd<cc[k])
    {
    try
      {
       p=pp[k-1]+(pp[k]-pp[k-1])*(codd-cc[k-1])/(cc[k]-cc[k-1]);
      }catch(...){
                   p=codd;
                  }
//   if(p>99999999.999) p=99999999.999;
      return p;
     }
  try
  {  
   p=pp[pn-1]+(pp[pn-1]-pp[pn-2])*(codd-cc[pn-1])/(cc[pn-1]-cc[pn-2]);
  }catch(...){
    p=codd;
            }
  //�������������ӵ�����
  //-----------------
//  if(p>99999999.999) p=99999999.999;
   return p;
  }
   return codd;
 }

//------------------------------------------------------------------------------
long us_sig(int sign,long val) //���ݳ���С��/����16bits . ע�⣡����
  {
	if((val&(1<<(sign-1)))==0)
	   return val;
	 else
           return (val|(~((1<<sign)-1)));
	}

//---------------------------------------------------------------------------
 long us_sig1(int sign,long val)  //���ݳ���С��/����32bits . ע�⣡����
  {
   unsigned long cd,sw;
    int i;
	if((val&(1<<(sign-1)))==0)
		return val;
	 else
         {
          sw=0;
          for(i=0;i<sign-1;i++) sw=sw+(int)pow(2,i);
           cd=(~val+1);
           cd=cd&sw;
          return cd*(-1);
         }

  }
 //---------------------------------------------------------------------------
long us_sigx(int sign,long val)  // ���λΪ�����ţ�����λΪ��ֵ
   {
      unsigned long cd,sw;
      int i;
      if((val&(1<<(sign-1)))==0)
		return val;
      else
      {
          sw=0;
          for(i=0;i<sign-1;i++)
              sw=sw+(int)pow(2,i);
          cd=val&sw;
          return cd*(-1);
      }
   }

//------------------------------------------------------------------------------
float fcs_32s(unsigned long fcs_vcod,int numb,int stre) //�з��źϲ���32λ�ɿ� ����У��
{
	float  codd;
        union cxn_32bit_node {
                float        bit32_i;
                unsigned long int   bit32_c;
                } fcs32bit;
        fcs32bit.bit32_c= fcs_vcod;//-16777216;
	codd= fcs32bit.bit32_i;

        if( stricmp(extpar[numb].kind[stre],"poly")==0 )//if(kn==2) //��������Ϊ��a0+x(a1+x(a2+x(a3+x(a4+x(a5+a6x)))))  extpar[numb].coefficient[j][heaTemp]
                codd=( extpar[numb].coefficient[stre][0]+codd*(extpar[numb].coefficient[stre][1]+codd*(extpar[numb].coefficient[stre][2]
                        +codd*(extpar[numb].coefficient[stre][3]+codd*(extpar[numb].coefficient[stre][4]+codd*extpar[numb].coefficient[stre][5])))) );
 	return(codd);
}
//------------------------------------------------------------------------------
float fcs_16s(unsigned short int fcs_vcod,int numb,int stre) //�з��źϲ���16λ�ɿ� ����У��
{
	short int  codd;
        float codf;
        union cxn_16bit_node {
                short int        bit16_i;
                unsigned short int   bit16_c;
                } fcs16bit;
        fcs16bit.bit16_c= fcs_vcod;//-16777216;
	codd= fcs16bit.bit16_i;

        if( stricmp(extpar[numb].kind[stre],"poly")==0 )//if(kn==2) //��������Ϊ��a0+x(a1+x(a2+x(a3+x(a4+x(a5+a6x)))))  extpar[numb].coefficient[j][heaTemp]
                codf=( extpar[numb].coefficient[stre][0]+codd*(extpar[numb].coefficient[stre][1]+codd*(extpar[numb].coefficient[stre][2]
                                +codd*(extpar[numb].coefficient[stre][3]+codd*(extpar[numb].coefficient[stre][4]+codd*extpar[numb].coefficient[stre][5])))) );
        else
                codf=codd;
        return(codf);
}
//------------------------------------------------------------------------------
//����UMA2000ʱ�����
void cal_time(__int64 t1,__int64 t2,__int64 t3,int uma_t[4])
{
  __int64 sum_mic,summ,noms,uma_d;

  sum_mic=t1; sum_mic=sum_mic<<16;
  sum_mic=sum_mic+t2;
  sum_mic=sum_mic*10;

/*  sum_mic=t1*65536*10;           //���� ��
  sum_mic=sum_mic+t2*10;
*/
  sum_mic=sum_mic+t3/1000;      //�ܺ�������
  summ=sum_mic;

  uma_t[0]=sum_mic/3600000;
  if(uma_t[0]>24)
  {
   uma_d=uma_t[0]/24;
   summ=summ-uma_d*24*3600000;
   uma_t[0]=summ/3600000;
  }

  sum_mic=summ-uma_t[0]*3600000;
  uma_t[1]=sum_mic/60000;

  noms=uma_t[0];
  noms=noms*3600000;
  sum_mic=summ-noms;
  noms=uma_t[1];
  noms=noms*60000;
  sum_mic=sum_mic-noms;
  uma_t[2]=sum_mic/1000;
  if(uma_t[2]>59)
 {
   ;// ShowMessage(IntToStr(t1)+":"+IntToStr(t2)+":"+IntToStr(t3)); 
  }
  noms=uma_t[0];
  noms=noms*3600000;
  sum_mic=summ-noms;
  noms=uma_t[1];
  noms=noms*60000;
  sum_mic=sum_mic-noms;
  noms=uma_t[2];
  noms=noms*1000;
  sum_mic=sum_mic-noms;
  uma_t[3]=sum_mic;

  return;
}
//------------------------------------------------------------------------------
