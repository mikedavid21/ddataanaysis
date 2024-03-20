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
#define RstreamNum 8   //可以处理的数据流数为 RstreamNum,从0开始.
#define RparaNum  6000 //RstreamNum*1000  //可以处理的总参数个数

//---------------------------------------------------------------------------
 struct headstruct
      {
        //修改同步字信息**********[10]可以处理10个流数据*****************************
        int frdepth[RstreamNum],frlength[RstreamNum],frpersec[RstreamNum],buffn[RstreamNum];   // 短周数/长周  ,段周字数,每秒长周数,缓冲区大小;
        int idnum[RstreamNum],id_sbit[RstreamNum],id_len[RstreamNum];   //ID字位置， //ID字开始有效位，//ID字有效位长度,从零开始记数;
        int time_mode[RstreamNum];//0-- DM6 format; 1-- BCD code(High 8 bits); 2-- BCD  code (Low 8Bits);
        int stream,parameter_number[RstreamNum];      //流数，参数个数
      };
  struct headstruct heads;

  struct parastruct      //每个参数的信息
      {
        char name[RstreamNum][50];      // 参数名
        int  wordl[RstreamNum];         //字长
        char  kind[RstreamNum][20];          // 校准类型
        //校准类型  poly(多项式)  hyper(双曲线) segm(点对) no(无) 布尔量校准(bool)  外部函数校准(userfun)
        /*      0: 普通校准
                1：开关量
                2：a*cod+b    // phy[0]=a; phy[1]=b; cod[0]=0: 无符号数
                                                   1: 有符号数（符号为在最高位）
                3: a/cod+b    //phy[0]=a; phy[1]=b;
        */

        //  int  invert;     // 参数是否反转 0： No,  1: Yes
        int  fraddr[RstreamNum];  // 短周号
        int  fraddr1[RstreamNum],fraddr2[RstreamNum],fraddr3[RstreamNum];

        int  wdaddr[RstreamNum];  //字号
        int  wdaddr1[RstreamNum],wdaddr2[RstreamNum],wdaddr3[RstreamNum];

        int  bit_start[RstreamNum][6];   //  取位时的起始位：如： 0，0，1，4
        int  bit_len[RstreamNum][6];     //  取位时的长度：  如： 12，8，4，4
        int  bit_object[RstreamNum][6];  //目标位置

        int cyl[RstreamNum];     //长周采样率
        int lutn[RstreamNum];     //校线点数
        int cod[RstreamNum][34];
        double phy[RstreamNum][34];

        //*新加*
        int word_interval[RstreamNum];// 字间隔
        double coefficient[RstreamNum][6];//poly(多项式)系数
        double segment[RstreamNum][8],segment_point[5];//双曲线；
        char code_type[RstreamNum][10];//原码类型;
      };
  struct parastruct *para;
  struct parastruct extpar[1004]; //para单个流最大所有参数个数，extpar每个流最大处理参数个数

  struct infstruct        //处理信息文件
       {
        char SYSINPUT[20];          //@SYSINPUT@     //保留字：SYSINPUT，表示系统特定输入，这些信息是固定的。
        char Planename[20];        //            //飞机名（字符）
        char Planeno[20];          //04             //飞机号（字符）
        char FlightDate[20];       //2002-11-24     //飞行日期（字符）
        char FlightNo[20];         //1              //飞行架次（字符）
        char MeasureSystem[20];    //DM5            //测试系统（字符）
        char DataProperty[20];     //PCM            //数据属性（字符）
        float Weight;              //-1             //飞机起飞重量（实型）
        float Core;                //-1             //起飞重心（实型）
        char Hang[50];              //-1             //起飞外挂（字符）
        float Temperature;         //-1             //场温（实型）
        float Press;               //-1             //场压（实型）
        float Windspeed;           //-1             //风速（实型）
        float View;                //-1             //能见度（实型）
        char Ground[50];            //-1             //场地（字符）
        char Order[50];             //-1             //科目（字符）
        char Testperson[20];        //-1             //员（字符）
        char FlightDataFile[1000];   //-1             //数据文件名称(带绝对路径) （字符）
        char DataHeadfile[1000];     //-1             //校准文件名称(带绝对路径) （字符）
        char SubjectName[50];       //111            //数据处理科目名称（字符）
        char UserID[20];            //ZZ             //数据处理用户名称（字符）
        char ProcessDate[20];       //2007-06-13     //数据处理日期（字符）
        char SYSINPUTEND[20];       //@SYSINPUTEND@  //保留字：SYSINPUTEND，特定输入结束
        char PARINFOR[20];          //@PARINFOR@           //保留字：ParInfor，处理参数信息

        char Par_name[RparaNum][50];            //参数名(字符)
        char Par_unit[RparaNum][20];            //单位(字符)
        float Par_UP[RparaNum];                //上限（实型）
        float Par_down[RparaNum];              //下限（实型）
        char Par_Group[RparaNum][15];                //参数组通道（字符）
        char Par_note[RparaNum][50];                //参数备注（字符）
        /*
        ANINDI&D_1 -1 -1     //参数名(字符) 单位(字符) 上限（实型） 下限（实型） 参数组通道（字符） 参数备注（字符） ，每个信息都是以空格隔开。
        -1
        * -1
        ANOR*B_1 -1 -1
        -1
        * -1
        LCMC@A_1 -1 -1
        -1
        * -1
        */
        char PARINFOREND[50];       //@PARINFOREND@                //保留字：ParINforEND，参数信息结束
        char TIMEINFO[20];          //@TIMEINFO@                   //保留字：TimeInfo，时间信息

        //10 1 1 10 11 11 1   N个时间段
        //开始时间小时（整型） 开始时间分钟（整型） 开始时间秒（整型） 结束时间小时（整型） 结束时间分钟（整型） 结束时间秒（整型） 采样率（整型），每个信息都是以空格隔开
        //char Par_name[RparaNum][50];            //参数名(字符)  输出顺序
        char Par_order[RparaNum][50];           //参数顺序      中间顺序
        int order[RparaNum];                        //参数顺序位子

        int BegH[100];
        int BegM[100];
        int BegS[100];
        int EndH[100];
        int EndM[100];
        int EndS[100];
        int Rate[100]; 
        int BegTime[100];   //新加
        int EndTime[100];   //新加

        char TIMEINFOREND[20];      //@TIMEINFOREND@  int TIMEINFOREND;             //保留字：TIMEINFOREND，时间段信息结束
        char SYSOUTPUT[20];         //@SYSOUTPUT@                  //保留字：SYSOUTPUT，系统特定输出信息
        //ZZ20070613DM5PCM161958.eng   //输出数据文件名称（文件格式见后），该文件命名要和任务号一致
        //ZZ20070613DM5PCM161958.cod
        //ZZ20070613DM5PCM161958.sta     .eng(工程量)，.cod(码值)，.sta(处理状态)
        char FilePath[300];
        char SYSOUTPUTEND[20];     //@SYSOUTPUTEND@               //保留字：SYSOUTPUTEND，系统输入结束
        char INPUT[20];            //@INPUT@                      //保留字：Input，校准方法输入，用户可以根据需要进行定义
        char INPUTEND[20];         //@INPUTEND@                   //扩展输入
        char OUTPUT[20];           //@OUTPUT@                     //保留字：INPUTEND，输入结束
        char OUTPUTEND[20];        //@OUTPUTEND@                  //保留字：Output，校准方法输出，用户可以根据需要进行定义
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
 void cal_time(__int64,__int64,__int64,int*);  //计算UMA2000时间参数
//--------------------------------------------------------------------------------
 //*新加*
 int CheckBox2Checked=-1;
 int TimeId = -99 ;  //结束标志符
 int read_hea();
 int process(int streams); //流数,时间段数,流参数总数
 int read_infs();
 int unite();
 bool stop=false;
 float check[15]={0};     // check[9]：CGauge1->Progress的叠加;
                          // check[8]：  数据的真实的最后时间
                          // check[10];   计算无效时间段
                          // check[11];   计算第一个流提取中间文件的大小
                          // check[12];    计算第一个流提取中间文件的合并实时大小
                          // check[13];   判断原码和物理量合并大循环
                          // check[0] ;   传递最开始的inf_time的值
                          // check[1];    百分比
                          // check[14];   当前处理时间显示
                          //check[6]      判断当前时间段是否有数据输出.
                          //check[2]     uma2000判断

 int Progress=0; //百分比
 int timei;

 FILE *tfp[RstreamNum+1],*tfc[RstreamNum+1];   //RstreamNum个流
 int hour,minute,second,messcond;
 float mestime;

 FILE *inf,*TReng,*TRcod,*TRsta;
 AnsiString  SaveFileNamePath,Dir,Tdir,TAeng,TAcod,TAsta,TempFileName;
 AnsiString TempData,TempDir;
 AnsiString tfs;
 using namespace std;
 int inf_paranumber,inf_time,i=0,j;    //参数个数，时间段
 int inf_stream;
 int distributary[RstreamNum+1][1006],distribut[RstreamNum+1]={0}; //分流
 int stream_no;
 int ctime[6];
 int ErroTimetable=100;
 int Singlestr=0;
 int streamsnum=0,filenum[RstreamNum]={0};  //需处理的具体流号;
 int checkinf_time;
 int StreamMaxCyl=0; //处理参数的流中最大采样率的流数
 
 double result[RparaNum];
 int Nresult[RparaNum];
 int  process(int streams);   
//--------------------old--------------------
  int findp[1000]; //单个流的可以处理的最大参数个数
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
// 1.对每个流,按时间段在原始数据文件中提取相应参数值;
// 2.对只有一个流处理要求的请求,在提取参数时直接写出结果文件;
// 3.提取的参数值文件按流信息存放在临时文件夹下的相应流文件中,一个流中的所有时间段;
// 4.对流文件进行合并,同时合并所有流文件;
// 5.按时间段对所有流文件合并,并且同时输出;
// 6.结果文件中物理量按不同时间段分开放,码值放在一个文件中.
// 7.2011.1.7 INF 文件异常处理
/***********************************************************************************/
 clrscr(); //清屏工作 
 cout<<"  多PCM数据流数据处理开始......"<<endl;
 try
   {
   int nRetCode = 0;
   Dir=argv[1];
   if(Dir.Length() == 0)
        {
                cout<<"  系统无法定位信息文件，请联系管理员......"<<endl;
                return  nRetCode;
        }
   Tdir=ChangeFileExt(Dir,""); //临时文件目录
   SaveFileNamePath=ExtractFilePath(Tdir); //结果文件目录

   TAsta=ChangeFileExt(Tdir,".sta"); //日志文件
   if((TRsta=fopen(TAsta.c_str(),"wt"))==NULL)
       {
         cout<<"  无法创建日志文件，请联系管理员......" <<endl;
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
              cout<<"  信息文件错误，无法处理，请联系管理员......"<<endl;
              fprintf(TRsta,"%d\n",0);
              fprintf(TRsta,"%s","信息文件错误，无法处理");
              fprintf(TRsta,"%s","Eof");
              return nRetCode;
        }
   cout<<"  信息文件分析完成.....\n"<<endl;
   if(read_hea()==0){
              cout<<"  校线文件错误，无法加载，请联系管理员......"<<endl;
              fprintf(TRsta,"%d\n",0);
              fprintf(TRsta,"%s\n","校线文件错误，无法加载");
              fprintf(TRsta,"%s","Eof");
              return nRetCode;
              }
   cout<<"  校线文件加载完成.....\n"<<endl;

   int MaxCyl=0;
   //将需处理的参数的流最大采样率的挑选出来作为合并分析的标准时间流 
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
         cout<<"  无法创建物理量文件，请联系管理员......" <<endl;
         fprintf(TRsta,"%d\n",0);
         fprintf(TRsta,"%s\n","无法创建物理量文件");
         fprintf(TRsta,"%s","Eof");
         return  nRetCode;
       }

   TAcod=ChangeFileExt(Tdir,".cod");
   if((TRcod=fopen(TAcod.c_str(),"wb"))==NULL)
       {
         cout<<"  无法创建码值文件，请联系管理员......" <<endl;
         fprintf(TRsta,"%d\n",0);
         fprintf(TRsta,"%s\n","无法创建码值文件");
         fprintf(TRsta,"%s","Eof");
         return  nRetCode;
       }

  fwrite(&infs.Planename,sizeof(char)*20,1,TRcod);        //            //飞机名（字符）
  fwrite(&infs.Planeno,sizeof(char)*20,1,TRcod);          //04             //飞机号（字符）
  fwrite(&infs.FlightDate,sizeof(char)*20,1,TRcod);       //2002-11-24     //飞行日期（字符）
  fwrite(&infs.FlightNo,sizeof(char)*20,1,TRcod);         //1              //飞行架次（字符）
  fwrite(&infs.SubjectName,sizeof(char)*50,1,TRcod);       //111            //数据处理科目名称（字符）
  fwrite(&infs.MeasureSystem,sizeof(char)*20,1,TRcod);    //DM5            //测试系统（字符）
  fwrite(&infs.DataProperty,sizeof(char)*20,1,TRcod);     //PCM            //数据属性（字符）
    infs.Weight=10000;
    fwrite(&infs.Weight,sizeof(float),1, TRcod);     //飞机起飞重量（字符）

  fwrite(&infs.Core,sizeof(infs.Core),1,TRcod);                //-1             //起飞重心（实型
  fwrite(&infs.Hang,sizeof(char)*50,1,TRcod);              //-1             //起飞外挂（字符）
  fwrite(&infs.Temperature,sizeof(infs.Temperature),1,TRcod);         //-1             //场温（实型）
  fwrite(&infs.Press,sizeof(infs.Press),1,TRcod);               //-1             //场压（实型）
  fwrite(&infs.Windspeed,sizeof(infs.Windspeed),1,TRcod);           //-1             //风速（实型）
  fwrite(&infs.View,sizeof(infs.View),1,TRcod);                //-1             //能见度（实型）
  fwrite(&infs.Ground,sizeof(char)*50,1,TRcod);            //-1             //场地（字符）
  fwrite(&infs.Order,sizeof(char)*50,1,TRcod);             //-1             //科目（字符）
  fwrite(&infs.Testperson,sizeof(char)*20,1,TRcod);        //-1             //员（字符）


        fwrite(&infs.UserID,sizeof(char)*20,1,TRcod);        //数据处理用户名称（字符）
        fwrite(&infs.ProcessDate,sizeof(char)*50,1,TRcod);        //数据处理日期

  fwrite(&inf_paranumber,sizeof(int),1,TRcod);         //参数个数；
  for(i=0;i<inf_paranumber;i++) //参数信息
        {
                fwrite(&infs.Par_name[i],sizeof(char)*50,1,TRcod);     //参数名(字符)
                fwrite(&infs.Par_unit[i],sizeof(char)*20,1,TRcod);     //单位(字符)
                fwrite(&infs.Par_UP[i],sizeof(float),1,TRcod);
                fwrite(&infs.Par_down[i],sizeof(float),1,TRcod);
                fwrite(&infs.Par_Group[i],sizeof(char)*15,1,TRcod);
        }

  fwrite(&infs.Planename,sizeof(char)*20,1,TReng);        //            //飞机名（字符）
  fwrite(&infs.Planeno,sizeof(char)*20,1,TReng);          //04             //飞机号（字符）
  fwrite(&infs.FlightDate,sizeof(char)*20,1,TReng);       //2002-11-24     //飞行日期（字符）
  fwrite(&infs.FlightNo,sizeof(char)*20,1,TReng);         //1              //飞行架次（字符）
  fwrite(&infs.SubjectName,sizeof(char)*50,1,TReng);       //111            //数据处理科目名称（字符）
  fwrite(&infs.MeasureSystem,sizeof(char)*20,1,TReng);    //DM5            //测试系统（字符）
  fwrite(&infs.DataProperty,sizeof(char)*20,1,TReng);     //PCM            //数据属性（字符）
    infs.Weight=10000;
    fwrite(&infs.Weight,sizeof(float),1, TReng);     //飞机起飞重量（字符）
  fwrite(&infs.Core,sizeof(infs.Core),1,TReng);                //-1             //起飞重心（实型
  fwrite(&infs.Hang,sizeof(char)*50,1,TReng);              //-1             //起飞外挂（字符）
  fwrite(&infs.Temperature,sizeof(infs.Temperature),1,TReng);         //-1             //场温（实型）
  fwrite(&infs.Press,sizeof(infs.Press),1,TReng);               //-1             //场压（实型）
  fwrite(&infs.Windspeed,sizeof(infs.Windspeed),1,TReng);           //-1             //风速（实型）
  fwrite(&infs.View,sizeof(infs.View),1,TReng);                //-1             //能见度（实型）
  fwrite(&infs.Ground,sizeof(char)*50,1,TReng);            //-1             //场地（字符）
  fwrite(&infs.Order,sizeof(char)*50,1,TReng);             //-1             //科目（字符）
  fwrite(&infs.Testperson,sizeof(char)*20,1,TReng);        //-1             //员（字符）
       
        fwrite(&infs.UserID,sizeof(char)*20,1,TReng);        //数据处理用户名称（字符）
        fwrite(&infs.ProcessDate,sizeof(char)*50,1,TReng);        //数据处理日期

  fwrite(&inf_paranumber,sizeof(int),1,TReng);         //参数个数；
  for(int i=0;i<inf_paranumber;i++)
        {
                fwrite(&infs.Par_name[i],sizeof(char)*50,1,TReng);     //参数名(字符)
                fwrite(&infs.Par_unit[i],sizeof(char)*20,1,TReng);     //单位(字符)
                fwrite(&infs.Par_UP[i],sizeof(float),1,TReng);
                fwrite(&infs.Par_down[i],sizeof(float),1,TReng);
                fwrite(&infs.Par_Group[i],sizeof(char)*15,1,TReng);
        }

   //---------------------------process-----------------------------------------
   for( stream_no=0;stream_no<RstreamNum;stream_no++) //按流数处理
        if( distribut[stream_no]>0 )
                {
                        if( process(stream_no)==0 )
                                {
                                        cout<<"  提取数据错误，请联系管理员......"<<endl;
                                        fprintf(TRsta,"%d\n",0);
                                        fprintf(TRsta,"%s\n","提取数据错误.");
                                        fprintf(TRsta,"%s","Eof");
                                        return  nRetCode;
                                }
                }
   //----------------------------unite------------------------------------------
  //for(timesect=0;timesect<inf_time;timesect++){//按时间段处理合并
     if(Singlestr!=1)
        if(unite()==0 )
                {
                        cout<<"  合并数据错误,请联系管理员......"<<endl;
                        fprintf(TRsta,"%d\n",0);
                        fprintf(TRsta,"%s\n","合并数据错误.");
                        fprintf(TRsta,"%s","Eof");
                        return nRetCode;
                }
//---------------------------------------------------------------------------

   fprintf(TRsta,"%d\n",1);
   fprintf(TRsta,"%s\n","数据处理成功结束.");
   fprintf(TRsta,"%s","Eof");

   fclose(TReng);
   fclose(TRcod);
   fclose(TRsta);

    //clreol();
    // //c+printf("  Current percentage is: %d%\r", 100);
   cout<<"\n  数据处理成功结束......"<<endl;

   delete[] para;
   RemoveDir(Tdir);
   return nRetCode;
 }
 catch( ... )
 {
   cout<<"   程序异常结束，请联系管理员....."<<endl;
   exit(1);
 }

}
//------------------------------------------------------------------------------
int process(int streams)//流数,流参数总数
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
    cout<<"  正在提取第"<<(streams+1)<<"个流数据,请稍候........................"<<endl;
     // //c+printf("  Current percentage is: %d%\r", Progress);
    if((dp=fopen(TempData.c_str(),"rb"))==NULL)
            {                                                                                                                      
                    cout<<"  无法加载原始数据" <<(streams+1)<<"，请联系管理员......"<<endl;
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
                    cout<<"  第" <<(streams+1)<< "数据文件的" "时间参数名没有找到,请联系管理员......"<<endl;
                    return 0;
            }                                                                                                                      
    findt=0;

    //clreol();
   cout<<"  正在校验需处理参数......"<<endl;
    // //c+printf("  Current percentage is: %d%\r", Progress);

   for(j=0;j<distribut[streams];j++)
        findp[j]=0;
   for(i=0;i<par_count[streams];i++) //单流所有参数个数
        {
                for(j=0;j<distribut[streams];j++) //实际提取的单流参数个数
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
                        cout<<"  参数"<<infs.Par_name[distributary[streams][i]]<<" 没有找到，请联系管理员......"<<endl;
                        return 0;
                }

   ext_count=distribut[streams]; //单流实际处理参数个数
  //--------------------------------star processing-----------------------------
  /****************************************************************************
   每个frame_no短周号和 word_no字号均单独设成数组中去
   addr，addr1，addr2，addr3为总共四个字的提取，
                            为在一个长周中subcod的具体位置
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
     if(extpar[i].cyl[streams]<=heads.frdepth[streams])  // 正常采样/子采样参数
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
     else  //超采参数
       {
                npfr=extpar[i].cyl[streams]/heads.frdepth[streams];   //参数每短周出现次数
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


   //将地址矩阵addr各列按照最大值补齐。
   maxe=0;
   for(i=0;i<ext_count+4;i++)
     if(addr[0][i]>maxe) maxe=addr[0][i]; //max为所需处理参数的最大采用率
   for(i=0;i<ext_count+4;i++)
      {
        tsub=addr[0][i];// 每个参数的采样率
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
                                addr[k][i]=dd[j];        //补点，补成最大采样率
                                addr1[k][i]=dd1[j];
                                addr2[k][i]=dd2[j];
                                addr3[k][i]=dd3[j];
                        }
      }
      for(i=0;i<ext_count+4;i++) addr[0][i]=maxe;
    //seaching the begin time.
  int wadd,wadd1,sv,sv_error_start,second_mf;
  __int64    maskw,cd1,cd2,cd3,cd4,cdd,cdd1,cdd2,cdd3,cdd4; //4字节,32位 0~4294967295
  checkinf_time=inf_time;
  __int64 tc1,tc2,tc3;

  
   //clreol();
  cout<<"  正在查找时间................."<<endl;
   // //c+printf("  Current percentage is: %d%\r", Progress);

  if(Singlestr==1)
        {
                fp=TReng;
                if(CheckBox2Checked==0)
                        fc=TRcod;
        }
  else {
                askk=".eng_tmp"+IntToStr(streams);
                asfp=ChangeFileExt(TempFileName,askk);//物理量
                fp=fopen(asfp.c_str(),"w");
                askk=".cod_tmp"+IntToStr(streams);
                asfc=ChangeFileExt(TempFileName,askk);//原码
                if(CheckBox2Checked==0)
                        fc=fopen(asfc.c_str(),"w");
        }
  int  CheckFseek=100;
  //---------------按时间段处理-------------------
  for(int timetab=0;timetab<inf_time;timetab++)
        {
                                        //按时间段提取
                                        //infs.BegTime[inf_i],infs.EndTime[inf_i],infs.Rate[inf_i],
                                        //开始时间,结束时间,采样率,
                if( inf_time-checkinf_time!=0 && Singlestr ==1 ) //单流结束标示符
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
                
                if( Singlestr == 1 )    //单流直接输出
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
                sts=infs.BegTime[timetab];   //开始时间
                ets=infs.EndTime[timetab];   //结束时间
                extr=infs.Rate[timetab]; //采样率
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
                                        case 3: //秦川格式
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
                                        case 4: //KAM-500格式
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
                                        case 5: //770格式
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
                                          case 6: //进气道畸变采集器时间字格式    jqd
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
                                        case 7: //UMA2000采集器时间字格式
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
                                               cout<<"第"<<streams<<"个流原始数据格式错误,请联系管理员......"<<endl;
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
                  cout<<"第"<<timetab+1<<"个处理结束时间小于飞行开始时间，无法处理......"<<endl;
                  ErroTimetable=timetab;
                  goto lab1;//return 0;
                }


                if(sv>=sts && sv<=ets )
                {
                     break;   //当前时间大于提取开始时间，直接处理
                }


                }  //  if(read_subf()).....
        if(stop) return 0;
     }while( (sv!=sts) && (!feof(dp)) );
//---------------------------------------------------------
     if(sts>sv)
        {
                cout<<"  第"<<(timetab+1)<<"个时间段的处理开始时间存在跳点，无法处理，建议将处理的开始时间定为0:0:0......"<<endl;
                ErroTimetable=timetab;
                goto lab1;//return 0;
        }

     if(extr>heads.frpersec[streams]*maxe)
        {
            cout<<"  第"<<(timetab+1)<<"个时间段的提取采样率太大,将按照最大长周采样率处理......"<<endl;

         }
        int bch,sc;  //隔BCH个长周处理一个
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
                          case 3: //秦川格式
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
                         case 4: //KAM-500格式
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
                        case 5: //770格式
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
                        case 6: //进气道畸变采集器时间字格式  jqd
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
                        case 7: //UMA2000采集器时间字格式
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
                                cout<<"第"<<streams<<"个流原始数据时间格式错误,请联系管理员......"<<endl;
                                return 0;
                    }
                if(ctime[0]>23) ctime[0]=0;
                if(ctime[1]>59) ctime[1]=0;
                if(ctime[2]>59) ctime[2]=0;

                sv=ctime[0]*3600+ctime[1]*60+ctime[2];   //sv 时间飞行时间
                if(sv>=(ctime[0]*3600+ctime[1]*60+ctime[2])&& timei==0)
                        {
                                printf("  处理开始时间：%02d:%02d:%02d\n\r",ctime[0],ctime[1],ctime[2]);
                                timei=ctime[0]*3600+ctime[1]*60+ctime[2];
                                check[14]=timei;
                        }
               if(sv>(check[14]+59+270))
                        {
                                printf("  当前处理时间：%02d:%02d:%02d\n\r",ctime[0],ctime[1],ctime[2]);
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
                                case 1: //单字参数
                                        maskw=0;
                                        s_bit=extpar[i].bit_start[streams][0];
                                        l_bit=extpar[i].bit_start[streams][0]+extpar[i].bit_len[streams][0];
                                        for(j=s_bit;j<l_bit;j++)
                                                maskw=maskw+(short)pow(2,j);
                                        vcod[i]=(subcod[wadd]&maskw)>>s_bit;
                                        vphy[i]=lut(vcod[i],i,streams);
                                        break;
                                case 2://双字参数
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

                                        if(stricmp(extpar[i].kind[streams],"rs232")==0)      //20091015加入
                                                {
                                                        vcod[i]=cd1;
                                                        vphy[i]=(cd1*100+cd2)/100.0;
                                                        break;
                                                }

                                        if( (extpar[i].bit_object[streams][0]+extpar[i].bit_object[streams][1])==0  ) //兼容最早没有填写的目标位参数的提取
                                                cdd=cd1<<extpar[i].bit_len[streams][1];
                                        else
                                                {
                                                        cdd=cd1<<extpar[i].bit_object[streams][0]; //向左移动  bit_object[0]位:起始位在 bit_object[0]位
                                                        cd2=cd2<<extpar[i].bit_object[streams][1];
                                                }
                                        cdd=cdd|cd2;
                                        vcod[i]=cdd;
                                        if( stricmp(extpar[i].name[streams],"U13_2_1_3_2")==0 ) //2011.4.29飞机所刘颖加
                                            {
                                               ppk=(cdd&0xFFFFFFFF)>>16;
                                               if( ppk==21764 )
                                                   { 
                                                       cdd= (vcod[i]&0xFFFF);
                                                       vphy[i]=lut(cdd,i,streams);
                                                   }
                                               break;
                                            }

                                       /* else if( stricmp(extpar[i].code_type[streams],"fcsf")==0 ) //fcs飞控16位有符号
                                             {
                                                   vphy[i]=fcs_16s((unsigned short int)vcod[i],i,streams);
                                                   break;
                                             }
                                        else if( stricmp(extpar[i].code_type[streams],"fcsf")==0 )//飞控32位有符号浮点
                                                        vphy[i]=fcs_32s(vcod[i],i,streams);

                                        */
                                       if( stricmp(extpar[i].code_type[streams],"fcsf")==0 )
                                        {
                                           if(extpar[i].bit_len[streams][0]+extpar[i].bit_len[streams][1]>16)
                                                  vphy[i]=fcs_32s( (unsigned long int)vcod[i],i,streams);      //fcs飞控32位有符号
                                           else
                                                  vphy[i]=fcs_16s((unsigned short int)vcod[i],i,streams); //fcs飞控16位有符号
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
                                                        cdd=cd1<<extpar[i].bit_object[streams][0]; //向左移动  bit_object[0]位:起始位在 bit_object[0]位
                                                        cdd1=cd2<<extpar[i].bit_object[streams][1];
                                                        cdd2=cd3<<extpar[i].bit_object[streams][2];
                                                        cdd=cdd|cdd1; cdd=cdd|cdd2;
                                                }
                                        vcod[i]=cdd;

                                        if( stricmp(extpar[i].code_type[streams],"fcsf")==0 )
                                        {
                                           if(extpar[i].bit_len[streams][0]+extpar[i].bit_len[streams][1]+extpar[i].bit_len[streams][2]>16)
                                                  vphy[i]=fcs_32s( (unsigned long int)vcod[i],i,streams);      //fcs飞控32位有符号
                                           else
                                                  vphy[i]=fcs_16s((unsigned short int)vcod[i],i,streams); //fcs飞控16位有符号
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
                                                                cdd1=cd1<<extpar[i].bit_object[streams][0]; //向左移动  bit_object[0]位:起始位在 bit_object[0]位
                                                                cdd2=cd2<<extpar[i].bit_object[streams][1];
                                                                cdd3=cd3<<extpar[i].bit_object[streams][2];
                                                                cdd4=cd4<<extpar[i].bit_object[streams][3];
                                                                cdd=cdd|cdd1; cdd=cdd|cdd2;cdd=cdd|cdd3;cdd=cdd|cdd4;
                                                        }
                                                vcod[i]=cdd;
                                                if( stricmp(extpar[i].code_type[streams],"fcsf")==0 )
                                                {
                                                   if(extpar[i].bit_len[streams][0]+extpar[i].bit_len[streams][1]+extpar[i].bit_len[streams][2]+extpar[i].bit_len[streams][3]>16)
                                                        vphy[i]=fcs_32s( (unsigned long int)vcod[i],i,streams);      //fcs飞控32位有符号
                                                   else
                                                        vphy[i]=fcs_16s((unsigned short int)vcod[i],i,streams); //fcs飞控16位有符号
                                                   break;
                                                }
                                                vphy[i]=lut(vcod[i],i,streams);
                                        break;
                                default:
                                         vcod[i]=0;
                                         vphy[i]=0.0;
                                         cout<<extpar[i].name[streams]<<"  校线参数位数定义错误，该参数处理结果错误，请联系管理员......"<<endl;
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

                                fprintf(fp,"%02d:%02d:%02d:%03d   ",ctime[0],ctime[1],ctime[2],ctime[3]);  //结果文件
                                if(CheckBox2Checked==0)
                                        fprintf(fc,"%02d:%02d:%02d:%03d   ",ctime[0],ctime[1],ctime[2],ctime[3]);   //码值文件
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
        if( ErroTimetable!=timetab  ) //超出实际时间段的不输出
                {
                        if( timetab!=inf_time-1 && Singlestr!=1)//最后一个时间段或者单流均 不输出
                                {
                                        fprintf(fp,"%3d:%3d:%3d:%3d\n",TimeId,TimeId,TimeId,TimeId);  //时间段结束标志
                                        if(CheckBox2Checked==0)
                                                fprintf(fc,"%3d:%3d:%3d:%3d\n",TimeId,TimeId,TimeId,TimeId);   //时间段结束标志
                                }
                        //gotoxy(wherex()-24, wherey()-1);
                         //clreol();
                        //cout<<"  处理结束时间：" <<ctime[0]<< ":"<<ctime[1]<<":"<<ctime[2]<<endl;
                        printf("  处理结束时间：%02d:%02d:%02d\n\r",ctime[0],ctime[1],ctime[2]);
                         // //c+printf("  Current percentage is: %d%\r", Progress);

                }
        Progress=(int) ( check[9]+ 80.0/( streamsnum*inf_time ) ); //每个流的每个时间段处理结束时应该达到的百分比
        check[9]=check[9]+ 80.0/( streamsnum*inf_time );

        if(sv<ets)
                {
                        cout<<"  第"<<(timetab+1)<<"个时间段数据文件结束,未到达结束时间......."<<endl;
                        if ( (sts-sv-10)*2*heads.frdepth[streams]*heads.frlength[streams]*heads.frpersec[streams]!=0 )
                                check[8]= ctime[0]*3600+ctime[1]*60+ctime[2];  //数据的真实的最后时间
                }
        else
                {
                         //clreol();
                        cout<<"  第"<<(timetab+1)<<"个时间段数据提取结束......"<<endl;
                         // //c+printf("  Current percentage is: %d%\r", Progress);
                }

  }//按时间段处理
  if( Singlestr ==1 ) //单流结束标示符
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

//---------------------------按时间段处理---------------------------------------
 fclose(dp);
 return 1;  
 }
 catch ( ... )
        {
                cout<<"  提取数据异常,请联系管理员......"<<endl;
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
   for(i=0;i<RstreamNum;i++)//按流打开临时结果文件      RstreamNum
   {
     if(distribut[i]>0)
        {
                //cout<<"  校准第" <<(i+1)<<"个流数据,请稍候......"<<endl;

                askk=".eng_tmp"+IntToStr(i);
                asfp=ChangeFileExt(TempFileName,askk);//物理量

                askk=".cod_tmp"+IntToStr(i);
                asfc=ChangeFileExt(TempFileName,askk);//原码
                if((tfp[i]=fopen(asfp.c_str(),"rt+"))==NULL)
                        {
                                cout<<"  无法加载物理量数据" <<(i+1)+ "，请联系管理员......"<<endl;
                                return 0;
                        }
                if(CheckBox2Checked==0)
                    {
                        if((tfc[i]=fopen(asfc.c_str(),"rt+"))==NULL)
                                {
                                        cout<<"  无法加载原码数据" <<(i+1)<<"，请联系管理员......"<<endl;
                                        return 0;
                                }
                    }
                //第四个流为高采数据
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
   inf_time=check[0]-check[10]/streamsnum;//真实有效时间段
   checkinf_time=inf_time;


   //******合并streams****************

   int h[RstreamNum+1],m[RstreamNum+1],sec[RstreamNum+1],mes[RstreamNum+1],times[RstreamNum+1],timest[RstreamNum+1];   //物理量时间和原码时间一致

  do
    {
       //*************合并变量5个流***********
   double ** data ,** datat,** datar, **TempResult ;        // 物理量STEP 1: SET UP THE ROWS.
   data = new double*[RstreamNum+1] ;
   datat = new double*[RstreamNum+1] ;
   datar = new double*[RstreamNum+1] ;
   TempResult= new double*[RstreamNum+1] ;

   for (j = 0; j < RstreamNum+1; j++)    //new 数组数不能为变量
        {
                data[j] = new double[1000];
                datat[j] = new double[1000];
                datar[j] = new double[1000];
                TempResult[j] = new double[1000];
        }
    //*************合并原码,原码和物理量时间是一致的*****************
     if( check[13]!=0 )
        {
                check[6]=0;
                checkinf_time=inf_time; //合并原码时，checkinf_time check[6]回到最初值
                if(CheckBox2Checked==0)
                   {
                        for( i=0;i<streamsnum;i++ )
                                tfp[filenum[i]]=tfc[filenum[i]];
                        tfp[RstreamNum]=tfc[RstreamNum];
                   }
        }
     check[13]++; //判断只循环2次
     //*************

      NextTime:   //*****************************时间段节点********************

      if(check[6]==0 && (inf_time+1-checkinf_time)!=1)
                printf("  第%d个时间段提取的各个数据流时间无法匹配,有效时间段太短,请修改时间段或采样率!\n\r",inf_time-checkinf_time);
      check[6]=0;
      if(check[13]==1 )
                cout<<"  正在合并第" <<(inf_time+1-checkinf_time)<< "个时间段物理量数据,请稍候......"<<endl;
      if(check[13]==2 )
         if(CheckBox2Checked==0)
                cout<<"  正在合并第" <<(inf_time+1-checkinf_time)<< "个时间段原码数据,请稍候......"<<endl;
         else
                break;

      //---------------十进制输出----分时间段输出多段时间段结果文件，原码不分开放-------------------------------
      if( check[13]==1 ) //***物理量***
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
      if( check[13]==2 ) //***原码***
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
      //---------------十进制输出----分时间段输出多段时间段结果文件，原码不分开放-------------------------------
      //************t1 t2 t3初值归0***************
      for (i = 0; i < RstreamNum+1; i++)
        {
                times[i]=0;
                timest[i]=0;

        }
      //**************读取每个流 T1***************
      for(i=0;i<streamsnum;i++) //读顺序的流,向顺序RstreamNum流靠近
        {
                //*********************物理量**************************************
                fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                times[filenum[i]]= (h[filenum[i]]*3600+m[filenum[i]]*60+sec[filenum[i]])*1000+mes[filenum[i]];
                for(j=0;j<distribut[filenum[i]];j++)  //distribut[filenum[i]] 参数数
                        fscanf(tfp[filenum[i]],"%lf",&data[filenum[i]][j]);// filenum[i] 流数
        }

     //**************读取 T3,使得t3>=t1***************
     for(i=0;i<streamsnum;i++)
       while(times[RstreamNum]<times[filenum[i]])    //t3<t1
        {
                if( feof(tfp[RstreamNum])  && checkinf_time==inf_time )   //所有的时间段都有问题
                        {
                                cout<<"第"<<filenum[0]<<"个流数据时间有问题,请联系管理员......"<<endl;
                                return 0;
                        }
                if( feof(tfp[RstreamNum]) )  goto lab2;
                fscanf(tfp[RstreamNum],"%d:%d:%d:%d", &h[RstreamNum],&m[RstreamNum],&sec[RstreamNum],&mes[RstreamNum]);
                times[RstreamNum]= (h[RstreamNum]*3600+m[RstreamNum]*60+sec[RstreamNum])*1000+mes[RstreamNum];

                 //************************第(checkinf_time>1)个时间段的第一个符合(t3>t1)的参数选取时，异常结束处理****************************************
                if( h[RstreamNum]==TimeId && m[RstreamNum]==TimeId && sec[RstreamNum]==TimeId && mes[RstreamNum]==TimeId )//判断是否有时间段的结束标志
                                {
                                        if(feof(tfp[RstreamNum]))  goto lab2;
                                        //**************所有流数据统一到同一时间段的结束标志处**********
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
                                                                                //fscanf(tfc[filenum[i]],"%lf",&Codata[filenum[i]][j]); //***原码***
                                                                        }
                                                        }
                                        checkinf_time=checkinf_time-1;
                                        if(checkinf_time!=0) goto  NextTime;
                                } 
                for(j=0;j<distribut[RstreamNum];j++)
                                fscanf(tfp[RstreamNum],"%lf",&data[RstreamNum][j]);
          }
      //**************读取每个流 T2,使得t2>t3***************
      for(i=0;i<streamsnum;i++)
        while(times[RstreamNum]>timest[filenum[i]]) //t3>t2
                {
                        if( feof(tfp[filenum[i]]) && checkinf_time==inf_time )
                                {
                                        cout<<"第"<<filenum[i]<<"个流数据时间有问题,请联系管理员......"<<endl;
                                        return 0;
                                }
                        if( feof(tfp[filenum[i]]) )  goto lab2;
                        fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                        timest[filenum[i]]= (h[filenum[i]]*3600+m[filenum[i]]*60+sec[filenum[i]])*1000+mes[filenum[i]];

                        //************************第(checkinf_time>1)个时间段的第一个符合(t3<t2)的参数选取时，异常结束处理****************************************
                        if( h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                {
                                        if(feof(tfp[filenum[i]]))    goto lab2;
                                        //**************所有流数据统一到同一时间段的结束标志处**********
                                        for(i=0;i<streamsnum;i++)
                                        while(h[filenum[i]]!=TimeId || m[filenum[i]]!=TimeId || sec[filenum[i]]!=TimeId || mes[filenum[i]]!=TimeId)
                                                {
                                                        if(feof(tfp[filenum[i]]))  goto lab2;   //结束插值处理
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
                                        if(checkinf_time!=0) goto  NextTime;    //开始下一个时间段的插值处理
                                }
                          //************************
                          
                        for(j=0;j<distribut[filenum[i]];j++)
                                fscanf(tfp[filenum[i]],"%lf",&datat[filenum[i]][j]);

                        //找最近的两个点，交换n1/t1 ==  n2/t2
                        if(times[RstreamNum]>timest[filenum[i]])
                            {
                                //***************交换t1=t2********************
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

   //***************对每个流的每个参数进行插值计算*******************************
         for(i=0;i<streamsnum;i++)
             {
                if( (times[RstreamNum]<=timest[filenum[i]]) && (times[RstreamNum]>=times[filenum[i]]) ) //if((t3<=t2)&&(t3>=t1))
                       {
                         //插值.
                         //if(filenum[i]==4)
                         if(filenum[i]==StreamMaxCyl)
                            {
                                for(j=0;j<distribut[filenum[i]];j++)
                                   {
                                      result[ summation ]= data[RstreamNum][j];
                                      summation++;   //计数
                                   }
                            }
                         else
                            {
                                 for(j=0;j<distribut[filenum[i]];j++)
                                    {
                                        //20110421线性差值修改为逼近差值
                                        //datar[filenum[i]][j]=(datat[filenum[i]][j] -data[filenum[i]][j])*times[RstreamNum]/(timest[filenum[i]]-times[filenum[i]])+(data[filenum[i]][j] *timest[filenum[i]]-datat[filenum[i]][j]*times[filenum[i]])/(timest[filenum[i]]-times[filenum[i]]);
                                        if( fabs(timest[filenum[i]]-times[RstreamNum]) <  fabs(times[RstreamNum]-times[filenum[i]]) )
                                              //t2-t3 < t3-t1 逼近取n2
                                                datar[filenum[i]][j]=datat[filenum[i]][j];
                                        else//( fabs(timest[filenum[i]]-times[filenum[0]]) >  fabs(times[filenum[0]]-times[filenum[i]]) )
                                             //t2-t3 >=t3-t1 逼近取n1
                                                datar[filenum[i]][j]=data[filenum[i]][j];

                                        //高采样在前的方式合并
                                        //避免取值在1/2处取得重复逼近值，取消DeiDa T的逼近方式获得的数据值
                                       if(fabs(timest[filenum[i]]-times[RstreamNum]) ==  fabs(times[RstreamNum]-times[filenum[i]])
                                          && heads.frdepth[StreamMaxCyl]*heads.frpersec[StreamMaxCyl]/(heads.frdepth[filenum[i]]*heads.frpersec[filenum[i]])<2  )

                                          {
                                           if(datar[filenum[i]][j]==TempResult[filenum[i]][j] )
                                                {
                                                    datar[filenum[i]][j]=datat[filenum[i]][j];
                                                }
                                           }
                                        result[ summation ]=  datar[filenum[i]][j];
                                        summation++;   //计数
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
                                        //将结果行所有的值都记录下来，以备比较
                                        TempResult[filenum[i]][j]= datar[filenum[i]][j];
                                   }
                             }   
                        //*********print out******************
                        if(p!=-1)
                           {
                                 check[6]++;
                                 if( check[13]==1 ) //***物理量***
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
                                 if( check[13]==2 )  //***原码***
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

                        if(h[RstreamNum]==TimeId && m[RstreamNum]==TimeId && sec[RstreamNum]==TimeId && mes[RstreamNum]==TimeId )//判断是否有时间段的结束标志
                                {

                                        if(feof(tfp[RstreamNum]))  goto lab2;

                                        //**************所有流数据统一到同一时间段的结束标志处**********
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
                        //***************避免T3跳点   t3<t2  and t3<t1读取下一个T3********************
                        for(i=0;i<streamsnum;i++)
                            while( times[RstreamNum]<times[filenum[i]] ) //   t3<t1 ->  next t3  保证t3>=t1
                                {
                                        if(feof(tfp[RstreamNum]))   goto lab2;
                                        p=fscanf(tfp[RstreamNum],"%d:%d:%d:%d", &h[RstreamNum],&m[RstreamNum],&sec[RstreamNum],&mes[RstreamNum]);
                                        times[RstreamNum]= (h[RstreamNum]*3600+m[RstreamNum]*60+sec[RstreamNum])*1000+mes[RstreamNum];

                                        if(h[RstreamNum]==TimeId && m[RstreamNum]==TimeId && sec[RstreamNum]==TimeId && mes[RstreamNum]==TimeId )//判断是否有时间段的结束标志
                                                {
                                                        if(feof(tfp[RstreamNum]))  goto lab2;

                                                        //**************所有流数据统一到同一时间段的结束标志处**********
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
                        while( times[RstreamNum]>timest[filenum[i]] )//t3>t2 ->next t2 保证t3<=t2
                             {
                                //***************交换t1=t2********************
                                times[filenum[i]]=timest[filenum[i]];
                                for(j=0;j<distribut[filenum[i]];j++)
                                        data[filenum[i]][j]=datat[filenum[i]][j];

                                if(feof(tfp[filenum[i]]))    goto lab2;
                                fscanf(tfp[filenum[i]],"%d:%d:%d:%d", &h[filenum[i]],&m[filenum[i]],&sec[filenum[i]],&mes[filenum[i]]);
                                timest[filenum[i]]= (h[filenum[i]]*3600+m[filenum[i]]*60+sec[filenum[i]])*1000+mes[filenum[i]];

                                if(h[filenum[i]]==TimeId && m[filenum[i]]==TimeId && sec[filenum[i]]==TimeId && mes[filenum[i]]==TimeId )
                                     {
                                        if(feof(tfp[filenum[i]]))    goto lab2;
                                        //**************所有流数据统一到同一时间段的结束标志处**********
                                        for(i=0;i<streamsnum;i++)
                                            while(h[filenum[i]]!=TimeId || m[filenum[i]]!=TimeId || sec[filenum[i]]!=TimeId || mes[filenum[i]]!=TimeId)
                                                {
                                                        if(feof(tfp[filenum[i]]))  goto lab2;   //结束插值处理
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
                                        if(checkinf_time!=0) goto  NextTime;    //开始下一个时间段的插值处理
                                     }

                                for(j=0;j<distribut[filenum[i]];j++)
                                        fscanf(tfp[filenum[i]],"%lf",&datat[filenum[i]][j]);
                             }
                  }//for(i=0;i<streamsnum;i++)
           } // else
        } //while(1)

   lab2: 
   if(check[6]==0)
        printf("  第%d个时间段提取的各个数据流时间无法匹配,有效时间段太短,请修改,请修改时间段或采样率!\n\r",inf_time-checkinf_time+1);
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
                                DeleteFile( ChangeFileExt(TempFileName,".cod_tmp"+IntToStr(filenum[i])) );   //***原码***
                        }
                   }
              else
                        DeleteFile( ChangeFileExt(TempFileName,".eng_tmp"+IntToStr(filenum[i])) );   //删除流结果中间文件
         }
  }while(check[13]==1);

  for(i=0;i<4;i++)  //最后一个时间段
        {
                fwrite(&TimeId,4,1,TReng);
                if(CheckBox2Checked==0)
                        fwrite(&TimeId,4,1,TRcod);
        }
   //clreol();
  cout<<"  数据合并结束，正在输出数据，请稍侯......\n"<<endl;

  //fclose(Test);
  return 1;
  }
  catch( ... )
        {
                cout<<"  结果输出异常，请联系管理员......"<<endl;
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
             cout<<"  信息文件无法打开!"<<endl;
              Beep(100,100);
              return 0;
             }
  cout<<"\n  正在分析信息文件......"<<endl;
  fscanf(inf,"%s \n",infs.SYSINPUT);
  if(stricmp(infs.SYSINPUT,"@SYSINPUT@")!=0)
           return 0;
  fscanf(inf,"%s \n",infs.Planename);
  fscanf(inf,"%s \n",infs.Planeno);
  fscanf(inf,"%s \n",infs.FlightDate);
  fscanf(inf,"%s \n",infs.FlightNo);
  fscanf(inf,"%s \n",infs.MeasureSystem); //测试系统（字符）
  fscanf(inf,"%s \n",infs.DataProperty);
  fscanf(inf,"%lf \n",&infs.Weight);      //飞机起飞重量（实型）
  fscanf(inf,"%f \n",&infs.Core);
  fscanf(inf,"%s \n",infs.Hang);
  fscanf(inf,"%f \n",&infs.Temperature);
  fscanf(inf,"%f \n",&infs.Press);       //场压（实型）
  fscanf(inf,"%f \n",&infs.Windspeed);
  fscanf(inf,"%f \n",&infs.View);
  fscanf(inf,"%s \n",infs.Ground);
  fscanf(inf,"%s \n",infs.Order);
  fscanf(inf,"%s \n",infs.Testperson);
  
  fscanf(inf,"%s \n",infs.FlightDataFile); //数据文件名称(带绝对路径) （字符）
  fscanf(inf,"%s \n",infs.DataHeadfile);    //校准文件名称(带绝对路径) （字符）

  fscanf(inf,"%s \n",infs.SubjectName);
  fscanf(inf,"%s \n",infs.UserID);          //数据处理用户名称（字符）
  fscanf(inf,"%s \n",infs.ProcessDate);
  fscanf(inf,"%s \n",infs.SYSINPUTEND);     //保留字：SYSINPUTEND，特定输入结束
  if(stricmp(infs.SYSINPUTEND,"@SYSINPUTEND@")!=0)
           return 0;

  fscanf(inf,"%s \n",infs.PARINFOR);      // @PARINFOR@   //保留字：ParInfor，处理参数信息
  if(stricmp(infs.PARINFOR,"@PARINFOR@")!=0){
           cout<<"  信息文件中无处理参数信息......"<<endl;
           return 0;
           }
  i=0;
  fscanf(inf,"%s \n",infs.Par_name[i]);     //参数名(字符)
  fscanf(inf,"%s \n",infs.Par_unit[i]);     //单位(字符)
  fscanf(inf,"%lf \n",&infs.Par_UP[i]);
  fscanf(inf,"%lf \n",&infs.Par_down[i]);
  fscanf(inf,"%s \n",infs.Par_Group[i]);
  fscanf(inf,"%s \n",infs.Par_note[i]);     //参数备注（字符）
  fscanf(inf,"%s \n",infs.PARINFOREND);  //@PARINFOREND@  //保留字：ParINforEND，参数信息结束
  while(stricmp(infs.PARINFOREND,"@PARINFOREND@")!=0)
        {
                i++;
                strcpy(infs.Par_name[i],infs.PARINFOREND);
                if(strlen(infs.Par_name[i])<3)
                {
                   cout<<"  数据处理列表参数信息INF文件生成错误，无法处理！"<<endl;
                   return  0;
                }
                fscanf(inf,"%s \n",infs.Par_unit[i]);     //单位(字符)
                fscanf(inf,"%lf \n",&infs.Par_UP[i]);     //上限（实型）
                fscanf(inf,"%lf \n",&infs.Par_down[i]);   //下限（实型）
                fscanf(inf,"%s \n",infs.Par_Group[i]);    //参数组通道（字符）
                fscanf(inf,"%s \n",infs.Par_note[i]);     //参数备注（字符）
                fscanf(inf,"%s \n",infs.PARINFOREND);
        }
        
  /*****************对参数信息进行归纳处理*******************************************/
  inf_paranumber=i+1; //选取的参数总数
  k=0;
  for(j=0;j<RstreamNum;j++)
        {
                filenum[j]=0;
                for(i=0;i<inf_paranumber;i++)
                        {
                                inf_stream=StrToInt( infs.Par_name[i][strlen(infs.Par_name[i])-1] );
                                if(inf_stream>RstreamNum || inf_stream==0)
                                        {
                                                cout<<"  提取得参数不是流参数，请检查！"<<endl;
                                                return  0;
                                        }
                                //************重复处理参数提示******************
                                for( int jj=0;jj<i;jj++ )
                                        {
                                                if( stricmp( infs.Par_name[i],infs.Par_name[jj])==0 )
                                                        {
                                                                cout<<"  "<<infs.Par_name[i]<<"重复选取，请修改。"<<endl;
                                                                return  0;
                                                        }
                                        }
                                //***********记录参数位子*********************
                                if( inf_stream==(j+1) )
                                        {
                                                strcpy(infs.Par_order[k] ,infs.Par_name[i]); //把每个流的参数放在连续中
                                                k++;
                                                distributary[j][distribut[j]]=i;
                                                distribut[j]++;    //单流参数总数
                                                if(distribut[j]>1000)
                                                        {
                                                                cout<<" 处理单流参数太多,超过1000个,不能处理..."<<endl;
                                                                return 0;
                                                        }
                                        }
                          }
          }
  //************计算需处理的流总数以及具体的流****************
   streamsnum=0;
   for(i=0;i<RstreamNum;i++)
        {
                if(distribut[i]>0)
                        {
                                filenum[streamsnum]=i;  //需处理的具体流号
                                streamsnum++;   //需处理流总数
                        }
        }
   if( streamsnum<1 )  return 0;
   //************
   if(streamsnum==1)
        Singlestr=1; //单流处理

   //----------------------infs.order[j]记录每个参数在中间infs.Par_order的位子---
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
                cout<<"  信息文件中无时间段......"<<endl;
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
                cout<<"  时间段时间信息超出范围错误，请确定......"<<endl;
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
                                cout<<"  时间段错误或者时间段超过100，请确定......"<<endl;
                                return 0;
                        }
                if(infs.EndH[i]>24) infs.EndH[i]=24;
                if(infs.EndM[i]>60) infs.EndM[i]=60;
                infs.BegTime[i]=3600*infs.BegH[i]+ 60*infs.BegM[i]+ infs.BegS[i];
                infs.EndTime[i]=3600*infs.EndH[i]+ 60*infs.EndM[i]+ infs.EndS[i];
     }
  inf_time=i+1;//一共的时间段
  check[0]= inf_time;
  fscanf(inf,"%s \n",infs.SYSOUTPUT); //@SYSOUTPUT@                  //保留字：SYSOUTPUT，系统特定输出信息
  fscanf(inf,"%s \n",infs.FilePath);
  fscanf(inf,"%d\n",&CheckBox2Checked);  //是否输出源码 -1 否； 0 是。
  fscanf(inf,"%s \n",infs.SYSOUTPUTEND);

  fscanf(inf,"%s \n",infs.INPUT);
  fscanf(inf,"%s \n",infs.INPUTEND);
  fscanf(inf,"%s \n",infs.OUTPUT);
  fscanf(inf,"%s \n",infs.OUTPUTEND);
 /* if(strcmp( "@OUTPUTEND@",infs.OUTPUTEND)!=0)
  {
     cout<<"  数据处理列表参数信息INF文件生成错误，无法处理！"<<endl;
     return  0;
  }*/
  fclose(inf);
  return 1;

  }
 catch ( ... )
        {
                cout<<"  信息文件异常,请联系管理员......"<<endl;
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

    double heaTempDouble;  //新加入Hea_Temp
    char heaTempChar[50];  //新加入Hea_Temp
    int heaTemp,start[50];
    double efficient[50];

    all_count=0;
    head_fname=infs.DataHeadfile;
    if((hp=fopen(head_fname.c_str(),"r"))==NULL)
        {
              cout<<"  带头文件无法打开!"<<endl;
              Beep(100,100);
              return 0;
        }
    cout<<"\n  正在加载校线文件......"<<endl;

//新加入头信息
            fscanf(hp,"%s \n",nouse);
            fscanf(hp,"%s \n",nouse);
            fscanf(hp,"%s %s\n",nouse,heaTempChar);
            fscanf(hp,"%s %s\n",nouse,heaTempChar);

            AnsiString ss= heaTempChar;
            ss=UpperCase(ss.SubString(1,3));  //判断是否为arj21飞机

            fscanf(hp,"%s %d\n",nouse,&heads.stream);  //流数
            if( heads.stream>RstreamNum )
                {
                        cout<<"  数据流不能大于"<<RstreamNum<<"，请联系管理员......"<<endl;
                        return 0;
                }
            fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);
             MaxSingleParaNmu=0;
            for(j=0;j<heads.stream;j++){
                    for(i=0;i<6;i++)   fscanf(hp,"%s \n",nouse);  //The format description of stream 1:
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %d\n",nouse,&heads.parameter_number[j]);   //参数个数
                    if(MaxSingleParaNmu<heads.parameter_number[j])
                          MaxSingleParaNmu=heads.parameter_number[j];
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);   //位速率
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);
                    fscanf(hp,"%s %d\n",nouse,&heads.id_len[j]);//PCM字长
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    if(stricmp(strlwr(heaTempChar),"kam500")==0)
                          heads.time_mode[j]=4;
                    else if(stricmp(heaTempChar,"770")==0)
                          heads.time_mode[j]=5;
                    else if(stricmp(heaTempChar,"秦川")==0)
                          heads.time_mode[j]=3;
                    else if(stricmp(strlwr(heaTempChar),"uma2000")==0)
                          heads.time_mode[j]=7;
                    else if(strcmp(strlwr(heaTempChar),"jqd")==0)
                          heads.time_mode[j]=6;
                    else
                          heads.time_mode[j]=0;   //if(stricmp(heaTempChar,"dm6")==0)
                    fscanf(hp,"%s %d\n",nouse,&heads.frlength[j]);  //短周字数
                    fscanf(hp,"%s %d\n",nouse,&heads.frdepth[j]);  //短周数/长周
                    fscanf(hp,"%s %d\n",nouse,&heads.frpersec[j]);// 长周数/秒
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);  //同步字长度
                    fscanf(hp,"%s %s\n",nouse,heaTempChar); //同步字偏吗
                    fscanf(hp,"%s %d\n",nouse,&heads.idnum[j]); // id word address
                    fscanf(hp,"%s %d\n",nouse,&heads.id_sbit[j]); // id word start bit
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);//时间插入
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);   //out信号极性
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);   //输入位
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);   //in信号极性
                    fscanf(hp,"%s :%s\n",nouse,heaTempChar);    //位顺序
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);   //同步策列
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble); //同步字位置
                    fscanf(hp,"%s %s\n",nouse,heaTempChar);      //子桢方式
                    fscanf(hp,"%s %s %s\n",nouse,heaTempChar,heaTempChar);  //ID计数方向
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);    //ID开始值
                    fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);  //循环同步值  0
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
                 fscanf(hp,"%s\n",para[i].kind[j]);//校准类型  poly(多项式)  hyper(双曲线) segm(点对) no(无) 布尔量校准(bool)  外部函数校准(userfun)
                 fscanf(hp,"%s %s\n",nouse,para[i].code_type[j]); //原玛类型  bcd uint(无符号)  sint（有符号） fcsf(16/32飞控浮点)
                 fscanf(hp,"%s %s\n",nouse,heaTempChar);
                 while(stricmp(nouse,"rate:")!=0)
                               fscanf(hp,"%s ",nouse);
                 fscanf(hp," %d",&para[i].cyl[j]);// extrate value per sub f
                 fscanf(hp,"%s %d",nouse,&para[i].word_interval[j]); //*新加* 字间隔
                 fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);//符号位
                 fscanf(hp,"%s %lf\n",nouse,&heaTempDouble);//参数是否反转
                 while(stricmp(nouse,"pcm_words:")!=0)
                               fscanf(hp,"%s ",nouse);
                 fscanf(hp," %d\n",&para[i].wordl[j]); //parameter name

                 fscanf(hp,"%s %d,%d,%d,%d,%d,%d\n",nouse,&para[i].fraddr[j],&para[i].fraddr1[j],&para[i].fraddr2[j],&para[i].fraddr3[j],&heaTempDouble,&heaTempDouble); //frame address
                 fscanf(hp,"%s %d,%d,%d,%d,%d,%d\n",nouse,&para[i].wdaddr[j],&para[i].wdaddr1[j],&para[i].wdaddr2[j],&para[i].wdaddr3[j],&heaTempDouble,&heaTempDouble); //word address
                 fscanf(hp,"%s ",nouse);
                 for(heaTemp=0;heaTemp<6;heaTemp++){
                          fscanf(hp,"%d, ",&start[heaTemp]);
                          para[i].bit_start[j][heaTemp]=start[heaTemp];    //  取位时的起始位：
                          }
                 fscanf(hp,"%s ",nouse);
                 for(heaTemp=0;heaTemp<6;heaTemp++){
                          fscanf(hp,"%d, ",&start[heaTemp]);
                          para[i].bit_len[j][heaTemp]=start[heaTemp];      //  取位时的长度：
                          }
                 fscanf(hp,"%s ",nouse);
                 for(heaTemp=0;heaTemp<6;heaTemp++){
                          fscanf(hp,"%d, ",&start[heaTemp]);
                          if(ss=="ARJ")
                                start[heaTemp]=0;
                          para[i].bit_object[j][heaTemp]=start[heaTemp];      //  取位时的目标位：
                          }                        
                 //fscanf(hp,"%s %lf,%lf,%lf,%lf,%lf,%lf\n",nouse,&heaTempDouble,&heaTempDouble,&heaTempDouble,&heaTempDouble,&heaTempDouble,&heaTempDouble);
                 fscanf(hp,"%s ",nouse);


                 if(stricmp(nouse,"coefficient:")==0){       //poly(多项式)
                         for(heaTemp=0;heaTemp<6;heaTemp++){
                             fscanf(hp,"%lf, ",&efficient[heaTemp]);
                             para[i].coefficient[j][heaTemp]= efficient[heaTemp];
                             }
                         }
                 else if(stricmp(nouse,"segment_number:")==0){    //segm(点对)
                         fscanf(hp,"%d ",&para[i].lutn[j]);
                         for(heaTemp=0;heaTemp<para[i].lutn[j];heaTemp++) {
                              fscanf(hp,"%s %d,%lf",nouse,&start[heaTemp],&efficient[heaTemp]);
                              para[i].cod[j][heaTemp]=start[heaTemp];
                              para[i].phy[j][heaTemp]=efficient[heaTemp];
                             }
                         }

                 else if(stricmp(nouse,"segment_point:")==0){    //hyper(双曲线)
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

               par_count[j]=i;         //每个流的数据个数
               all_count = all_count+par_count[j];   //总的数据个数
         }// for(j=1;j<stream+1;j++)
     } // while(!feof(hp)) 
     fclose(hp);
     return 1;  
 }
 catch ( ... )
        {
                cout<<"  校线文件异常,请联系管理员......"<<endl;
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
  nwfr=heads.frlength[strea];    //一个短周的短周字数
  idadd=heads.idnum[strea];
  maskw=0;
  for(i=0;i<heads.id_len[strea];i++)
        maskw=maskw+(short)pow(2,i);
  do
   {
        DatErro=DatErro+1;
        fread(&frame,2,nwfr,dp); ////一个短周，16-bit unsigned integer 为2字节
        if(DatErro>heads.frdepth[strea]*10)
                {
                        cout<<"  提示：数据长周连续丢失10个(如果连续提示，请检查数据和校线文件的匹配)......."<<endl;
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
                subcod[nwfr*j+i]=(frame[i]&0xffff); //一个短周
                     // subcod[nwfr*j+i]=subcod[frlength*frame_no+word_no]
        j++;
   }while((j<heads.frdepth[strea])&&(!feof(dp)));
   if(feof(dp))
        return 0;
   return 1;
  }
 catch ( ... )
 {
  cout<<"  短周数据异常,请联系管理员......"<<endl;
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

  //if(c>32768 && heads.time_mode[stre]==7)   //针对uma2000采集器校准
  //     c=c-65535;

  codd=c;


  if( stricmp(extpar[numb].code_type[stre],"fcs618")==0 ) //fcs 618特殊校准方式 20121008
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


  if(stricmp(extpar[numb].code_type[stre],"sint")==0)   //有符号参数，-(~para+1)         ??有符号参数处理
   {
    sigw=0;
    for(k=0;k<extpar[numb].wordl[stre];k++)
         sigw=sigw+extpar[numb].bit_len[stre][k];
    sigc=us_sig1(sigw,codd);
    codd=sigc;
   }
  //*******************************xint：最高位为符号位，1为负值，0为正值*****************
   if(strcmp(extpar[numb].code_type[stre],"xint")==0)   //有符号参数，+-(para)         ??有符号参数处理
  {   
        sigw=0;
        for(k=0;k<extpar[numb].wordl[stre];k++)
                   sigw=sigw+extpar[numb].bit_len[stre][k];
       sigc=us_sigx(sigw,codd);
       codd=sigc;
  }


  if(stricmp(kn,"poly")==0)//if(kn==2) //参数类型为：a0+x(a1+x(a2+x(a3+x(a4+x(a5+a6x)))))  extpar[numb].coefficient[j][heaTemp]
  {

    p=( extpar[numb].coefficient[stre][0]+codd*(extpar[numb].coefficient[stre][1]+codd*(extpar[numb].coefficient[stre][2]
        +codd*(extpar[numb].coefficient[stre][3]+codd*(extpar[numb].coefficient[stre][4]+codd*extpar[numb].coefficient[stre][5]))))  );
   return p;
  }
 if(stricmp(kn,"hyper")==0)//if(kn==3) //参数类型为：a/x+b; a/(b+cx)+d
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


 //针对码值是补码的情况
 //if(kn==4)  //码值有负值；
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

//点对校准  segm
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
  //特殊参数计算请加到下面
  //-----------------
//  if(p>99999999.999) p=99999999.999;
   return p;
  }
   return codd;
 }

//------------------------------------------------------------------------------
long us_sig(int sign,long val) //数据长度小于/等于16bits . 注意！！！
  {
	if((val&(1<<(sign-1)))==0)
	   return val;
	 else
           return (val|(~((1<<sign)-1)));
	}

//---------------------------------------------------------------------------
 long us_sig1(int sign,long val)  //数据长度小于/等于32bits . 注意！！！
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
long us_sigx(int sign,long val)  // 最高位为正负号，其它位为数值
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
float fcs_32s(unsigned long fcs_vcod,int numb,int stre) //有符号合并成32位飞控 并加校线
{
	float  codd;
        union cxn_32bit_node {
                float        bit32_i;
                unsigned long int   bit32_c;
                } fcs32bit;
        fcs32bit.bit32_c= fcs_vcod;//-16777216;
	codd= fcs32bit.bit32_i;

        if( stricmp(extpar[numb].kind[stre],"poly")==0 )//if(kn==2) //参数类型为：a0+x(a1+x(a2+x(a3+x(a4+x(a5+a6x)))))  extpar[numb].coefficient[j][heaTemp]
                codd=( extpar[numb].coefficient[stre][0]+codd*(extpar[numb].coefficient[stre][1]+codd*(extpar[numb].coefficient[stre][2]
                        +codd*(extpar[numb].coefficient[stre][3]+codd*(extpar[numb].coefficient[stre][4]+codd*extpar[numb].coefficient[stre][5])))) );
 	return(codd);
}
//------------------------------------------------------------------------------
float fcs_16s(unsigned short int fcs_vcod,int numb,int stre) //有符号合并成16位飞控 并加校线
{
	short int  codd;
        float codf;
        union cxn_16bit_node {
                short int        bit16_i;
                unsigned short int   bit16_c;
                } fcs16bit;
        fcs16bit.bit16_c= fcs_vcod;//-16777216;
	codd= fcs16bit.bit16_i;

        if( stricmp(extpar[numb].kind[stre],"poly")==0 )//if(kn==2) //参数类型为：a0+x(a1+x(a2+x(a3+x(a4+x(a5+a6x)))))  extpar[numb].coefficient[j][heaTemp]
                codf=( extpar[numb].coefficient[stre][0]+codd*(extpar[numb].coefficient[stre][1]+codd*(extpar[numb].coefficient[stre][2]
                                +codd*(extpar[numb].coefficient[stre][3]+codd*(extpar[numb].coefficient[stre][4]+codd*extpar[numb].coefficient[stre][5])))) );
        else
                codf=codd;
        return(codf);
}
//------------------------------------------------------------------------------
//计算UMA2000时间参数
void cal_time(__int64 t1,__int64 t2,__int64 t3,int uma_t[4])
{
  __int64 sum_mic,summ,noms,uma_d;

  sum_mic=t1; sum_mic=sum_mic<<16;
  sum_mic=sum_mic+t2;
  sum_mic=sum_mic*10;

/*  sum_mic=t1*65536*10;           //毫秒 ；
  sum_mic=sum_mic+t2*10;
*/
  sum_mic=sum_mic+t3/1000;      //总毫秒数；
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
