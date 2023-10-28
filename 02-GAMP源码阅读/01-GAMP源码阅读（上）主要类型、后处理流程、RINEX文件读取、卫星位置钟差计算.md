>原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning
![1698494049(1)](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1698494049(1).png)

[TOC]

## 一、GAMP 简介

### 1、程序概述

GAMP 全称 (**G**NSS  **A**nalysis software for **M**ulti-constellation and multi-frequency **P**recise positioning)，在 RTKLIB 的基础上，将一些多余的函数、代码简洁化，精简出后处理 PPP 部分，并对算法进行改进增强。简化后代码比 RTKLIB 原版还要简单，对初学者非常友好，在我接触过的导航定位开源程序中算是最简单的。使用也很方便，软件包里提供了 VS 工程，和组织好的配置文件、数据文件，简单改改文件路径就能算出结果。

![GAMP 系统结构图](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GAMP%20%E7%B3%BB%E7%BB%9F%E7%BB%93%E6%9E%84%E5%9B%BE.png)

### 2、工具箱介绍

* 

* MatPlot：

### 3、函数调用关系

![1688082358362](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1688082358362.png)

### 4、程序执行流程



## 二、基础类型定义

### 1、宏定义

大部分沿用 RTKLIB，做了少量拓展

![rtklib.h 宏定义](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/rtklib.h%2520%25E5%25AE%258F%25E5%25AE%259A%25E4%25B9%2589.png)

### 2、结构体定义

大部分沿用 RTKLIB，做了少量拓展

![rtklib.h结构体](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/rtklib.h%25E7%25BB%2593%25E6%259E%2584%25E4%25BD%2593.png)

### 3、矩阵、向量、最小二乘、卡尔曼滤波

* GAMP 中用 double 类型一维数组表示矩阵，不能自动识别矩阵的行列数，每次传矩阵的时候都要传入行数 n、列数 m。
* 用矩阵的时候要先 malloc 开辟空间，用完记得 free 释放空间。
* 要能熟练计算矩阵加减乘除转置。
* 矩阵求逆用的 LU 分解法，时间复杂度 $O^3$ ，对于大规模的矩阵，如果利用矩阵的稀疏性和对称性等特性，而且当使用不完全分解方法（例如，只计算到一定程度或使用截断技术）时，LU 分解的效率会更高。
* `matprint()` 很常用，调试的时候不好直接看的矩阵元素的值，得输出到终端或者文件再看。

![image-20231021091639437](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021091639437.png)

### 4、时间和字符串

* GAMP 中时间一般都以 `gtime_t` 类型存储，为了提高时间表示的精度，分开存 GPST 时间的整秒数和不足一秒的部分。
* 经常需要做年月日时分秒、周+周内秒、GPST 三种时间之间的转换；想输出北京时间的时候要加 8 小时。
* BDT、GLONASST 不怎么用，读完文件就转为 GPS 时间了。

![image-20231021090651319](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021090651319.png)

### 5、坐标系统

* ECI 用的很少，只在 `sunmoonpos()` 函数中计算太阳月亮时候用到了，不用太关注。
* ENU、ECEF、LLH 三套坐标系都频繁使用，要熟练掌握他们直接的转换，包括协方差的转换
* ENU 是局部相对坐标系，以某一个 LLH 坐标为原点，坐标转换的时候要传入这个 LLH 坐标。
* ENU 常用 `e`表示、ECEF 常用 `r` 表示、LLH 常用 `pos` 表示。

![image-20231021091756265](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021091756265.png)

### 6、卫星系统、观测值

* **卫星系统表示**：
  * 表示卫星系统的字母：GRECJIS。
  * 7 位二进制码表示，对应位写 1 表示有对应的系统，做与运算可加系统。
* **卫星的表示**：
  * 可以表示为各系统的卫星ID（系统缩写+PRN）：B02、C21。
  * 也可表示为连续的 satellite number。
* **观测值类型**：
  * **C**：伪距、**D**：多普勒、**L**：载波相位、**S**：载噪比。
  * `CODE_XXX`：观测值类型定义，用一串连续的数字表示。
  * `sigind_t`：表示每种卫星系统的载波类型和观测值类型 ，每种类型的系统其实对应的就是一个 `sigind_t` 结构体。

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024191221181.png)

### 7、配置选项

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB%25E9%2585%258D%25E7%25BD%25AE%25E9%2580%2589%25E9%25A1%25B9.png)



## 三、后处理

### 1、main()

程序从 `main.c` 的 `main()` 函数开始执行，整个程序都在 `t1=clock()` 和 `t2=clock()` 中执行，求得`t2-t1`为程序执行时间。`main()` 函数接收传入的命令行参数即 `gamp.cfg` 的文件路径，如果传入了参数，**调用 `proccfgfile()` 进行下一步处理**。

> * VS 中：在 项目属性-调试-命令行参数 中指定命令行参数。
> * Windows 的文件路径中一般用 `\`，且为了避免转义需要写成 `\\`。linux一般用 `/`。

```c
int main(int argc, char **argv)
{
	//char cfgfile[1000]="C:\\mannual_GAMP\\Examples\\2017244\\gamp.cfg";
    char *cfgfile;
	long t1,t2;

	t1=clock();

	if (argc==1) {
		printf("\n * The input command-line parameter indicating configure file is lost, please check it!\n");
		return 0;
	}
	else {
		cfgfile=argv[1];
	}

	// 调用 proccfgfile() 处理配置文件
	/* find processing configure file */
	proccfgfile(cfgfile);

	t2=clock();

	printf("\n * The total time for running the program: %6.3f seconds\n%c",(double)(t2-t1)/CLOCKS_PER_SEC,'\0');
	//printf("Press any key to exit!\n");
	//getchar();

	return 0;
}
```

### 2、proccfgfile()：处理配置文件

![image-20230929095355253](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929095355253.png)

`proccfgfile()` 函数先将  `PPP_Glo` 结构体初始化，将处理方式，输入输出文件路径赋空值。打开传入的 `gamp.cfg` 文件，获取观测值文件路径和处理方式，根据观测文件的数量调用对应的函数：

* 单个观测文件**调用 `procOneFile()` 进行下一步处理**；
* 如果有多个文件调用 `batchProc()` 进行批量处理，`batchProc()` 会打开文件夹，循环查找文件夹中的观测值O文件，**调用 `procOneFile()` 进行下一步处理**。

> 观测值 `O` 文件的后缀有两种，一种是直接 `.O` 结尾，一种是 `.ddO` 结尾，dd表示年份后两位。

```c
static int proccfgfile(char cfgfile[])
{
	FILE *fp=NULL;
	char *p,tmp[MAXSTRPATH]={'\0'};

	// 将  PPP_Glo 结构体初始化，将处理方式、输入输出文件路径赋空值
	//initialization
	PPP_Glo.prcType=-1;
	PPP_Glo.outFolder[0]='\0';
	PPP_Glo.inputPath[0]='\0';

	// 打开传入的 gamp.cfg 配置文件
	if ((fp=fopen(cfgfile,"r"))==NULL) {
		printf("*** ERROR: open configure file failed, please check it!\n");
		return 0;
	}

	while (!feof(fp)) {
		tmp[0]='\0';
		fgets(tmp,MAXSTRPATH,fp);
		if ((tmp!=NULL)&&(tmp[0]=='#')) continue;

		// 获取观测值文件路径和处理方式
		if (strstr(tmp,"obs file/folder")) {
			p=strrchr(tmp,'=');
			sscanf(p+1,"%d",&PPP_Glo.prcType);

			tmp[0]='\0';
			if (fgets(tmp,MAXSTRPATH,fp)) {
				p=strrchr(tmp,'=');
				sscanf(p+1,"%[^,]",PPP_Glo.inputPath);
				// 调用 trimSpace() 去除空格，调用 cutFilePathSep() 去除文件末尾的 /
				trimSpace(PPP_Glo.inputPath);
				cutFilePathSep(PPP_Glo.inputPath);
			}
			else {
				printf("*** ERROR: read obs files path error!");
				return 0;
			}
			break;
		}
	}
	fclose(fp);

	if (PPP_Glo.prcType<0||PPP_Glo.prcType>2) {
		printf("*** ERROR: read obs files path error!");
		return 0;
	}

	if (PPP_Glo.prcType==0)	// 单个观测文件调用 procOneFile() 进行下一步处理
		procOneFile(PPP_Glo.inputPath,cfgfile,0,1);
	else if (PPP_Glo.prcType==1) // 多个文件调用 batchProc() 进行批量处理
		batchProc(PPP_Glo.inputPath,cfgfile);

	return 1;
}
```

### 3、procOneFile()：处理单个观测值文件

![image-20230929101446209](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929101446209.png)

* 先调用 `preProc()` 预处理：通过调用 `initGlobal()` 初始化 `PPP_Glo` 结构体；调用 `getObsInfo()`  读取观测O文件的一部分，获取起止时间、文件版本、天线种类等基础信息；为 `filopt.inf`、`filopt.outf` 开辟内存空间。

* 调用 `readcfgFile()` 读取整个配置文件，通过 `strstr(line,"start_time")` 匹配处理选项，存储到 `prcOpt_Ex`、`prcopt`。

* 调用 `getFopt_auto()` ，通过调用 `findClkFile()`、`findNavFile()`，根据后缀名自动查找各种 PPP 解算所需的文件，将文件路径存到 `fopt->inf` 中。

* **调用 `gampPos()` 进行下一步处理**；处理结束，调用 `postProc()` 释放 `filopt.inf`、`filopt.outf` 内存空间。

```c
extern void procOneFile(char file[], char cfgfile[], int iT, int iN)
{
	procparam_t pparam;
	gtime_t t={0},ts=t,te=t ;
	long t1,t2;

	t1=clock();

	// 先调用 preProc() 预处理：
	// 其通过调用 initGlobal() 初始化 PPP_Glo 结构体
	// 调用 getObsInfo()  读取观测O文件的一部分，获取起止时间、文件版本、天线种类等基础信息
	// 为 filopt.inf、filopt.outf 开辟内存空间
	preProc(file,&pparam,&ts,&te);

	printf(" * Processing the %dth", iN);
	if (iT>0) printf("/%d", iT);
	printf(" ofile: %s\n", PPP_Glo.ofileName_ful);

	// 调用 readcfgFile() 读取整个配置文件，通过 strstr(line,"start_time") 匹配处理选项，存储到 prcOpt_Ex、prcopt
	//read configure file
	readcfgFile(cfgfile,&pparam.prcopt,&pparam.solopt,&pparam.filopt);

	//single-, dual- or triple-frequency?
	if (pparam.prcopt.ionoopt==IONOOPT_IF12||pparam.prcopt.ionoopt==IONOOPT_UC1) {
		if (pparam.prcopt.nf!=1) {
			printf("*** ERROR: Number of frequencies Error! Please set inpfrq=1.\n");
			return;
		}
	}
	if (pparam.prcopt.ionoopt==IONOOPT_UC12) {
		if (pparam.prcopt.nf!=2) {
			printf("*** ERROR: Number of frequencies Error! Please set inpfrq=2.\n");
			return;
		}
	}

	//processing time set
	if (!PPP_Glo.prcOpt_Ex.bTsSet) PPP_Glo.prcOpt_Ex.ts=ts;
	else if (timediff(ts,PPP_Glo.prcOpt_Ex.ts)>0) PPP_Glo.prcOpt_Ex.ts=ts;
	if (!PPP_Glo.prcOpt_Ex.bTeSet)	 PPP_Glo.prcOpt_Ex.te=te;
	else if (timediff(te,PPP_Glo.prcOpt_Ex.te)<0) PPP_Glo.prcOpt_Ex.te=te;

	// 调用 getFopt_auto() ，通过调用 findClkFile()、findNavFile()，根据后缀名自动查找各种 PPP 解算所需的文件，将文件路径存到 fopt->inf 中
	//automatically matches the corresponding files
	getFopt_auto(file,PPP_Glo.obsDir,ts,te,pparam.prcopt,pparam.solopt,&pparam.filopt);

	// 调用 gampPos() 进行下一步处理
	// post processing positioning
	gampPos(PPP_Glo.prcOpt_Ex.ts, PPP_Glo.prcOpt_Ex.te, 0.0, 0.0, 
		&pparam.prcopt,&pparam.solopt,&pparam.filopt);

	// 调用 postProc() 释放 filopt.inf、filopt.outf 内存空间
	postProc(pparam);

	t2=clock();

	sprintf(PPP_Glo.chMsg," * The program runs for %6.3f seconds\n%c",(double)(t2-t1)/CLOCKS_PER_SEC,'\0');
	outDebug(OUTWIN,OUTFIL,0);
	printf("/*****************************  OK  *****************************/\n\n\n");

	if (PPP_Glo.outFp[OFILE_DEBUG]) {
		fclose(PPP_Glo.outFp[OFILE_DEBUG]);
		PPP_Glo.outFp[OFILE_DEBUG]=NULL;
	}
}
```

### 4、gampPos()：开始后处理

![image-20230929095543042](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929095543042.png)

* 先调用 `outhead()` 写输出文件的文件头。
* 调用 `setcodepri()` 设置观测值优先级。
* 调用 `readdcb()`、`readobsnav()`、`readpreceph()` 等函数读取文件。

* 文件读取完之后，**调用 `execses()` 进行下一步处理**。
* 处理完之后调用 `freeobsnav()`、`freepreceph()` 释放内存空间。

```c
extern int gampPos(gtime_t ts, gtime_t te, double ti, double tu, prcopt_t *popt, 
	               const solopt_t *sopt, filopt_t *fopt)
{
    int i,j,stat=0,index[MAXINFILE]={0};

	// 先调用 outhead() 写输出文件的文件头
	/* write header to output file */
	if (!outhead(fopt->outf,popt,sopt,PPP_Glo.outFp,MAXOUTFILE)) 
		return 0;
    
	for (i=0;i<MAXOUTFILE;i++) {
		if (fopt->outf[i]&&strlen(fopt->outf[i])>2) 
			PPP_Glo.outFp[i]=openfile(fopt->outf[i]);
		else
			PPP_Glo.outFp[i]=NULL;
	}

	// 调用 setcodepri() 设置观测值优先级
	/* set rinex code priority for precise clock */
	if (PMODE_PPP_KINEMA<=popt->mode)
		setcodepri(SYS_GPS,1,popt->sateph==EPHOPT_PREC?"PYWC":"CPYW");

	// 调用 readdcb()、readobsnav()、readpreceph() 等函数读取文件
	/* read satellite antenna parameters */
	if (*fopt->antf&&!(readpcv(fopt->antf,&pcvss))) {
		printf("*** ERROR: no sat ant pcv in %s\n",fopt->antf);
		return -1;
	}

	/* read dcb parameters */
	for (i=0;i<MAXSAT;i++) for (j=0;j<3;j++) {
		navs.cbias[i][j]=0.0;
	}
	if (*fopt->p1p2dcbf) 
		readdcb(fopt->p1p2dcbf,&navs);
	if (*fopt->p1c1dcbf) 
		readdcb(fopt->p1c1dcbf,&navs);
	if (*fopt->p2c2dcbf) 
		readdcb(fopt->p2c2dcbf,&navs);
	if (*fopt->mgexdcbf&&(popt->navsys&SYS_CMP||popt->navsys&SYS_GAL))
		readdcb_mgex(fopt->mgexdcbf,&navs,PPP_Glo.prcOpt_Ex.ts);

	/* read erp data */
	if (*fopt->eopf) {
		if (!readerp(fopt->eopf,&navs.erp)) {
			printf("ERROR: no erp data %s\n",fopt->eopf);
		}
	}

	/* read ionosphere data file */
	if (*fopt->ionf&&(popt->ionoopt==IONOOPT_TEC||((popt->ionoopt==IONOOPT_UC1||popt->ionoopt==IONOOPT_UC12)&&
		PPP_Glo.prcOpt_Ex.ion_const)))
		readtec(fopt->ionf,&navs,1);

	for (i=0;i<MAXINFILE;i++) index[i]=i;

	/* read prec ephemeris */
	readpreceph(fopt->inf,MAXINFILE,popt,&navs);

	/* read obs and nav data */
	if (!readobsnav(ts,te,ti,fopt->inf,index,MAXINFILE,popt,&obss,&navs,stas)) {
		freeobsnav(&obss,&navs);
		return 0;
	}

	if (PPP_Glo.nEpoch<=1) {
		strcpy(PPP_Glo.chMsg,"PPP_Glo.nEpoch<=1!\n\0");
		printf("%s",PPP_Glo.chMsg);
		freeobsnav(&obss,&navs);

		return 0;
	}

	
	//read igs antex only once, and save the elements in 'pcvss'
	/* set antenna paramters */
	setpcv(obss.data[0].time,popt,&navs,&pcvss,&pcvss,stas);

	/* read ocean tide loading parameters */
	if (popt->mode>PMODE_SINGLE&&fopt->blqf) {
		readotl(popt,fopt->blqf,stas);
	}

	// 调用 execses() 进行下一步处理
	//next processing
	stat=execses(popt,sopt,fopt);

	// 处理完之后调用调用 freeobsnav()、freepreceph() 释放内存空间
	/* free obs and nav data */
	freeobsnav(&obss,&navs);
	/* free prec ephemeris and sbas data */
	freepreceph(&navs);
	/* free antenna parameters */
	if (pcvss.pcv) {free(pcvss.pcv); pcvss.pcv=NULL; pcvss.n=pcvss.nmax=0;}
	if (pcvsr.pcv) {free(pcvsr.pcv); pcvsr.pcv=NULL; pcvsr.n=pcvsr.nmax=0;}
	/* free erp data */
	if (navs.erp.data) {free(navs.erp.data); navs.erp.data=NULL; navs.erp.n=navs.erp.nmax=0;}

	if (PPP_Glo.outFp[OFILE_IPPP]) fprintf(PPP_Glo.outFp[OFILE_IPPP],"-PPP_BLOCK\n");
	for (i=0;i<MAXOUTFILE;i++) {
		if (i==OFILE_DEBUG) continue;
		if (PPP_Glo.outFp[i]) {
			fclose(PPP_Glo.outFp[i]);
			PPP_Glo.outFp[i]=NULL;
		}
	}

	return stat;
}
```

### 5、excses()：执行后处理解算

![image-20230929095710372](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929095710372.png)

* 先调用 `sampledetermine()` 获取观测值采用间隔（解算频率）。
* 调用 `calCsThres()` 获取周跳检测的阈值，调用 `rtkinit()` 初始化 `rtk` 结构体。
* **调用 `procpos()` 进行下一步处理**。
* 处理完之后调用 `rtkfree()` 释放 `rtk` 结构体。

```c
static int execses(prcopt_t *popt, const solopt_t *sopt, filopt_t *fopt)
{
	rtk_t rtk;

	// 先调用 sampledetermine() 获取观测值采用间隔（解算频率），周跳检测会用到
	//to determine the thresh values for cycle slip detection
	PPP_Glo.sample=sampledetermine(popt);

	// 调用 calCsThres() 获取周跳检测的阈值
	if (fabs(PPP_Glo.prcOpt_Ex.csThresGF)<0.01||fabs(PPP_Glo.prcOpt_Ex.csThresMW)<0.01)
		calCsThres(popt,PPP_Glo.sample);

	// 调用 rtkinit() 初始化 rtk 结构体
	rtkinit(&rtk,popt);

	if (PPP_Glo.outFp[OFILE_IPPP]) outiPppHead(PPP_Glo.outFp[OFILE_IPPP],rtk);

	// 根据前向滤波、后向滤波、前后向结合，以不同的方式调用 procpos() 进行下一步处理
	if (popt->soltype==0) {       /* forward */
		PPP_Glo.revs=0;
		PPP_Glo.iObsu=0;
		PPP_Glo.iEpoch=0;

		
		procpos(&rtk,popt,sopt,0); 
	}
	else if (popt->soltype==1) {  /* backward */
		PPP_Glo.revs=1; 
		PPP_Glo.iObsu=obss.n-1;
		PPP_Glo.iEpoch=PPP_Glo.nEpoch;
		procpos(&rtk,popt,sopt,0);
	}
	else {   /* combined */
		PPP_Glo.solf=(sol_t *)malloc(sizeof(sol_t)*PPP_Glo.nEpoch);
		PPP_Glo.solb=(sol_t *)malloc(sizeof(sol_t)*PPP_Glo.nEpoch);
		if (PPP_Glo.solf&&PPP_Glo.solb) {

		}
		else
			printf("error : memory allocation");

		free(PPP_Glo.solf); PPP_Glo.solf=NULL;
		free(PPP_Glo.solb); PPP_Glo.solb=NULL;
	}

	// 处理完之后调用 rtkfree() 释放 rtk 结构体
	rtkfree(&rtk);
    
    return 0;
}
```

### 6、procpos()：进行定位解算

![image-20230929095813550](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929095813550.png)

* 循环调用 `inputobs()` 传入一个历元的观测值。
* 调用 `obsScan_SPP()` 观测值检测。
* 调用 `BDmultipathCorr()` 修正北斗伪距延迟。
* **调用 `rtkpos()` 进行下一步处理**。
* 处理完之后调用 `outResult()`、`outsol()` 输出结果。

```c
static void procpos(rtk_t *rtk,const prcopt_t *popt,const solopt_t *sopt,int mode)
{
	sol_t sol={{0}};
	gtime_t time={0};
	obsd_t obs[MAXOBS];
	int i,j,k=0,nep=0,nobs,n,solstatic,pri[]={0,1,2,3,4,5,1,6};

	solstatic=sopt->solstatic&&popt->mode==PMODE_PPP_STATIC;
	
	// 循环调用 inputobs() 传入一个历元的观测值
	/* processing epoch-wise */
	while ((nobs=inputobs(obs,obss,PPP_Glo.revs,&PPP_Glo.iObsu,&PPP_Glo.iEpoch))>=0) {
		PPP_Glo.tNow=obs[0].time;
		time2epoch(PPP_Glo.tNow,PPP_Glo.ctNow);
		sprintf(PPP_Glo.chTime,"%02.0f:%02.0f:%04.1f%c",PPP_Glo.ctNow[3],
			PPP_Glo.ctNow[4],PPP_Glo.ctNow[5],'\0');
		PPP_Glo.sowNow=time2gpst(PPP_Glo.tNow,NULL);

		k++;
		if (k==1) {
			for (j=0;j<MAXSAT;j++) {
				PPP_Glo.ssat_Ex[j].tLast=PPP_Glo.tNow;
			}
		}
        nep=(int)(30*(60/PPP_Glo.sample));
        if ((k-1)%nep==0) PPP_Glo.t_30min=PPP_Glo.tNow;

		// 调用 obsScan_SPP() 观测值检测
		//pseudorange observation checking
		obsScan_SPP(popt,obs,nobs,&n);
		if (n<=3) {
			sprintf(PPP_Glo.chMsg,"*** WARNING: There are only %d satellites observed, skip SPP!\n",n);
			outDebug(OUTWIN,OUTFIL,0);
			continue;
		}

		// 调用 BDmultipathCorr() 分段函数修正北斗伪距多路径延迟
		if (PPP_Glo.prcOpt_Ex.navSys&SYS_CMP) {
			BDmultipathCorr(rtk,obs,n);
		}

		// 调用 rtkpos() 进行逐历元解算
		i=rtkpos(rtk,obs,n,&navs);
		if (i==-1) rtk->sol.stat=SOLQ_NONE;
		else if (i==0) continue;

		// 解算完之后调用 outResult()、outsol() 输出结果
		if (mode==0) {  /* forward/backward */
			outResult(rtk,sopt);

			if (!solstatic&&PPP_Glo.outFp[0]) 
				outsol(PPP_Glo.outFp[0],&rtk->sol,sopt,PPP_Glo.iEpoch);
			else if (time.time==0||pri[rtk->sol.stat]<=pri[sol.stat]) {
				sol=rtk->sol;
				if (time.time==0||timediff(rtk->sol.time,time)<0.0) 
					time=rtk->sol.time;
			}
		}
		
	}
}
```

### 7、rtkpos()：逐历元解算

![image-20230929100123058](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100123058.png)

* 至此已经读完了文件，开始进行逐历元解算，先**调用 `spp()` 进行 SPP 解算**。
* 调用 `obsScan_PPP()` 观测值检测。
* 调用 `clkRepair()` 修复钟跳。
* **调用 `pppos()` 进行 PPP 解算**。
* 调用 `calDop()` 计算各种 DOP 值。
* 调用 `keepEpInfo()` 存储当前历元的信息，其中会调用 `gfmeas()`、`wlAmbMeas()`。

```c
static int rtkpos(rtk_t *rtk, obsd_t *obs, int n, const nav_t *nav)
{
	gtime_t time;
	int nu;
	char msg[128]="";
	prcopt_t *opt=&rtk->opt;

	rtk->sol.stat=SOLQ_NONE;
	time=rtk->sol.time;  /* previous epoch */
	PPP_Glo.bOKSPP=1;

	// 先调用 spp() 进行 SPP 解算流动站坐标，作为 PPP 初值
	/* rover position by single point positioning */
	if (!spp(obs,n,nav,opt,&rtk->sol,NULL,rtk->ssat,msg)) {
		sprintf(PPP_Glo.chMsg,"*** ERROR: point pos error (%s)\n",msg);
		outDebug(OUTWIN,OUTFIL,0);

		PPP_Glo.bOKSPP=0;
		PPP_Glo.nBadEpSPP++;

		//fewer than 4 satellites available, skip to next epoch
		if (n<=4) {
			return -1;
		}
	}
	
	// 计算和上一历元间时间间隔
	if (time.time!=0) rtk->tt=timediff(rtk->sol.time,time);

	// 调用 obsScan_PPP() 观测值检测
	nu=n;
	obsScan_PPP(opt,obs,n,&nu);
	if (nu<=4) {
		sprintf(PPP_Glo.chMsg, "*** WARNING: There are only %d satellites observed, skip PPP!\n",nu);
		outDebug(OUTWIN,OUTFIL,0);
		return 0;
	}
	
	// 调用 clkRepair() 修复钟跳
	//clock jump repair
	clkRepair(obs,nu);

	// 调用 pppos() 进行 PPP 解算
	/* precise point positioning */
	if (opt->mode>=PMODE_PPP_KINEMA) {
		pppos(rtk,obs,nu,nav);
	}
	else return 1;

	// 调用 calDop() 计算各种 DOP 值
	//calculate DOPs
	calDop(rtk,obs,nu);

	// 调用 keepEpInfo() 存储当前历元的信息，其中会调用 gfmeas()、wlAmbMeas()
	//save the information for current epoch
	keepEpInfo(rtk,obs,nu,nav);

	return 1;
}
```

## 四、RINEX 文件读取

### 1、readobsnav()：Rinex 文件读取主入口函数

```c
gtime_t ts                  解算开始时间
gtime_t te                  解算结束时间
double ti                   解算时间间隔
char **infile               传入文件路径数组
const int *index            对应文件下标
int n                       infile[]元素个数
const prcopt_t *prcopt      处理选项
obs_t *obs                  存观测数据OBS
nav_t *nav                  存导航电文数据NAV
sta_t *sta                  测站结构体，存obs文件头读取到的一部分的信息
```

> **index[]的作用**：会传给`execses_b()`，再传给`execses_r()`，再传给`execses()`，再传给`readobsnav() `。如果不需要根据 tu 分时间段解算，index 存的就是 0~n，如果需要分时间段解算，index 存的是对应时间段内文件的下标。



* 先初始化 obs、nav->eph、nav->geph；遍历 `infile[]`，如果下标和上一次循环的不同，记录当前`index[i]`值到`ind `。调用`readrnxt()`读取文件，其先调用 `readrnxfile()` 读取文件，如果测站名字为空，就给依据头文件自动赋 4 个字符的名字。

* 然后判断是否有观测数据和星历数据，成功读取到数据，就调用`sortobs()`，根据 time、rcv、sat ，对`obs->data`的元素进行排序、去重，得到历元数`nepoch`。

* 最后调用`uniqnav()`，其通过调用 `uniqeph()`、`uniqgeph()` 进行星历数据的排序去重，通过调用 `satwavelen()` 获取所有载波相位的波长到 `nav->lam。`

```c
static int readobsnav(gtime_t ts, gtime_t te, double ti, char *infile[MAXINFILE],
	                  const int *index, int n, const prcopt_t *prcopt,
	                  obs_t *obs, nav_t *nav, sta_t *sta)
{
	int i,j,ind=0,nobs=0,rcv=1,nep;

	// 初始化 obs、nav->eph、nav->geph
	obs->data=NULL; obs->n =obs->nmax =0;
	nav->eph =NULL; nav->n =nav->nmax =0;
	nav->geph=NULL; nav->ng=nav->ngmax=0;
	PPP_Glo.nEpoch=0;

	// 遍历 infile[]，调用readrnxt()读取文件
	for (i=0;i<n;i++) {
		// 如果下标和上一次循环的不同，记录当前index[i]值到ind
		if (index[i]!=ind) {
			if (obs->n>nobs) rcv++;
			ind=index[i]; nobs=obs->n; 
		}
		/* read rinex obs and nav file */
		nep=readrnxt(infile[i],rcv,ts,te,ti,prcopt->rnxopt,obs,nav,rcv<=2?sta+rcv-1:NULL);
	}

	// 判断是否有观测数据和星历数据
	if (obs->n<=0) {
		printf("*** ERROR: no obs data!\n");
		return 0;
	}
	if (nav->n<=0&&nav->ng<=0) {
		printf("*** ERROR: no nav data!\n");
		return 0;
	}

	// 调用sortobs()，根据 time、rcv、sat ，对 obs->data 的元素进行排序、去重，得到历元数nepoch
	/* sort observation data */
	PPP_Glo.nEpoch=sortobs(obs);

	// 最后调用uniqnav()，其通过调用 uniqeph()、uniqgeph() 进行星历数据的排序去重，
	// 通过调用 satwavelen() 获取所有载波相位的波长到 nav->lam。
	/* delete duplicated ephemeris */
	uniqnav(nav);

	/* set time span for progress display */
	if (ts.time==0||te.time==0) {
		for (i=0;   i<obs->n;i++) if (obs->data[i].rcv==1) break;
		for (j=obs->n-1;j>=0;j--) if (obs->data[j].rcv==1) break;
		if (i<j) {
			if (ts.time==0) ts=obs->data[i].time;
			if (te.time==0) te=obs->data[j].time;
			settspan(ts,te);
		}
	}

	// 判断有无 GLONASS 星历，为啥不放在前面？？？
	if (prcopt->navsys&SYS_GLO) {
		if (nav->ng<=0) {
			printf("*** ERROR: nav->ng<=0!\n");
		}
	}

	return 1;
}
```

```c
extern int readrnxt(const char *file, int rcv, gtime_t ts, gtime_t te,
                    double tint, const char *opt, obs_t *obs, nav_t *nav,
                    sta_t *sta)
{
    int i,stat=0;
    const char *p;
    char type=' ',*files[MAXEXFILE]={0};

    /*if (!*file) {
        return readrnxfp(stdin,ts,te,tint,opt,0,1,&type,obs,nav,sta);
    }
    for (i=0;i<MAXEXFILE;i++) {
        if (!(files[i]=(char *)malloc(1024))) {
            for (i--;i>=0;i--) free(files[i]);
            return -1;
        }
    }*/
    /* expand wild-card */
    /*if ((n=expath(file,files,MAXEXFILE))<=0) {
        for (i=0;i<MAXEXFILE;i++) free(files[i]);
        return 0;
    }*/
    /* read rinex files */
    //for (i=0;i<n&&stat>=0;i++) {
        // 调用 readrnxfile() 读取文件
        stat=readrnxfile(file,ts,te,tint,opt,0,rcv,&type,obs,nav,sta);
    //}

    // 如果测站名字为空，就给依据头文件自动赋 4 个字符的名字
    /* if station name empty, set 4-char name from file head */
    if (type=='O'&&sta) {
        if (!(p=strrchr(file,FILEPATHSEP))) p=file-1;
        if (!*sta->name) setstr(sta->name,p+1,4);
    }
    for (i=0;i<MAXEXFILE;i++) free(files[i]);
    
    return stat;
}
```

### 2、readrnxfile()：传入文件路径，读取起止时间内数据

* 如果传入了测站信息结构体 sta，调用 `init_sta()` 初始化，值赋 0，指针赋空。

* 根据文件名判断文件来源（COD、IGS、IGR、GFZ、ESA、IAC、其它），以此设置 index。

* 以读的方式打开解压后的文件，调用 `readrnxfp()` ，从文件描述符 fp 中读取文件，读完之后，关闭打开的文件描述符 `fp`。

```c
static int readrnxfile(const char *file, gtime_t ts, gtime_t te, double tint,
                       const char *opt, int flag, int index, char *type,
                       obs_t *obs, nav_t *nav, sta_t *sta)
{
    FILE *fp;
    int stat;
    //char tmpfile[1024];
    
    // 如果传入了测站信息结构体 sta，调用 init_sta() 初始化
    if (sta) init_sta(sta);
    
    // 判断文件名长度是否合理
    if ( strlen(file)<2 ) 
        return ' ';

    // 以读的方式打开解压后的文件
    if (!(fp=fopen(file,"r"))) {
        return ' ';
    }

    // 根据文件名判断文件来源（COD、IGS、IGR、GFZ、ESA、IAC、其它），以此设置 index。
    if (strstr(file,"cod")||strstr(file,"COD")) index=10;
    else if (strstr(file,"igs")||strstr(file,"IGS")) index=9;
    else if (strstr(file,"igr")||strstr(file,"IGR")) index=8;
    else if (strstr(file,"gfz")||strstr(file,"GFZ")) index=7;
    else if (strstr(file,"esa")||strstr(file,"ESA")) index=6;
    else if (strstr(file,"iac")||strstr(file,"IAC")) index=-1;
    else index=0;

    // 调用 readrnxfp() ，从文件描述符 fp 中读取文件
    /* read rinex file */
    stat=readrnxfp(fp,ts,te,tint,opt,flag,index,type,obs,nav,sta);
    
    // 读完之后，关闭打开的文件描述符 fp
    fclose(fp);
    
    /* delete temporary file */
    //if (cstat) remove(tmpfile);
    
    return stat;
}
```

### 4、readrnxfp()：传入文件描述符，调用对应的读取函数

* 调用 `readrnxh()` 读取头文件。并获取文件类型 `type`。
* 根据 `type` 调用对应的函数进行分类读取，`readrnxobs()` 读观测文件，`readrnxnav()`读星历文件，调用 `readrnxnav()` 读钟差文件。

```c
static int readrnxfp(FILE *fp, gtime_t ts, gtime_t te, double tint,
                     const char *opt, int flag, int index, char *type,
                     obs_t *obs, nav_t *nav, sta_t *sta)
{
    double ver;
    int sys,tsys;
    char tobs[NUMSYS][MAXOBSTYPE][4]={{""}};
    
    // 调用 readrnxh() 读取头文件。并获取文件类型 type
    /* read rinex header */
    if (!readrnxh(fp,&ver,type,&sys,&tsys,tobs,nav,sta)) return 0;
    
    // flag 置 0 就不读钟差
    /* flag=0:except for clock,1:clock */
    if ((!flag&&*type=='C')||(flag&&*type!='C')) return 0;
    
    // 根据 type 调用对应的函数进行分类读取
    /* read rinex body */
    switch (*type) {
        case 'O': return readrnxobs(fp,ts,te,tint,opt,index,ver,tsys,tobs,obs);
        case 'N': return readrnxnav(fp,opt,ver,sys    ,nav);
        case 'G': return readrnxnav(fp,opt,ver,SYS_GLO,nav);
        case 'J': return readrnxnav(fp,opt,ver,SYS_QZS,nav); /* extension */
        case 'L': return readrnxnav(fp,opt,ver,SYS_GAL,nav); /* extension */
        case 'C': return readrnxclk(fp,opt,index,nav);
    }

    return 0;
}
```

### 5、readrnxh()：读取文件头



* 函数的主体在一个 while 大循环中，循环读取每一行，直到出现 "END OF HEADER" 
* 首先进行第一行版本号读取，记录版本号、卫星系统以及观测文件类型。
* PGM / RUN BY / DATE 跳过不读。
* 
* 其它类型的行，根据文件类型，调用 `decode_obsh()`、`decode_navh()`、`decode_gnavh()`、`decode_hnavh()`、`decode_navh()` 读取。

![image-20231028161435181](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028161435181.png)

### 6、观测文件读取

#### 1. decode_obsh()：解析观测数据文件头

最关键的是解析观测值类型如下图，存到 `tobs` 三维数组中，【星座类型】【观测类型】【字符串数】 ,后面读文件体的时候要按文件头的观测值类型来读。

![image-20231028161101023](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028161101023.png)

#### 2. readrnxobs()：读取o文件中全部观测值数据

重复调用`readrnxobsb()`函数，直到所有的观测值全被读完，或者是出现了某个历元没有卫星的情况为止 

* 为`data[]` 开辟空间
* while大循环调用`readrnxobsb()`每次读取一个历元的观测数据，获取观测值数n
 * 遍历`data[]`，如果时间系统为UTC，转为GPST，调用`saveslips()`
* 调用`screent()`，判断传入的时间是否符合起始时间ts，结束时间te，时间间隔tint
* 遍历`data[]` ，调用`restslips()`，`addobsdata()`将`data[]`信息存到`obs`中 





#### 3. readrnxobsb()：读取一个观测历元的观测数据



* 调用`set_sysmask()`获取卫星系统掩码mask，mask在之后`decode_obsdata()`中会用到，mask中没有的卫星系统不用。

* 调用set_index()，将将tobs数组中存的观测值类型信息存到sigind_t类型的index[]结构体数组中，此时传入的tobs数组是二维数组，每个传入的tobs都存了一个卫星系统的观测值类型，同理index[]的一个元素就存一个卫星系统的所有观测值类型。

* while大循环，fgets()存一行的数据

  * 如果是第一行，则调用`decode_obsepoch()`函数解码首行数据（包括历元时刻、卫星数、卫星编号、历元状态等信息），并将信息保存 ，获取的卫星数量nsat是判断循环次数的关键。
  * 如果不是第一行则调用`decode_obsdata()`函数对该行观测数据进行数据解码，读取一个历元内一颗卫星的观测值 ，到data[n]
  * 知道读取数量 i 等于`decode_obsepoch()`获取的卫星数量`nsat`，结束循环，返回读取的观测值数（卫星数）

  

#### 4. decode_obsepoch()：解码历元首行数据

**2、3版本观测值文件有区别**：

- 2版本：

  ![image-20231028161530991](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028161530991.png)
  
  - 每历元首行数据前26位为历元时间（yy mm dd hh mm ss)，年是 2 位表示，str2time() 函数中可以把年的前两位也补上。
  - 29位epoch flag ，记录该历元状况，0表示正常，3:new site,4:header info,5:external event 
  - 30~32位为卫星数量
  - 33~68：各个卫星的PRN号，观测到的卫星数>12时，一行的信息存储不下会自动换行，并且卫星的PRN号与前一行对其 
  - 历元信息往下一行就是记录观测值的数据块，以每颗卫星为单位，依照头文件中的观测值类型及顺序，从左到右依次排列，每行记录5个观测值，一行不够时转下行。当所有卫星数据记录完后，转到下一个历元。 观测值的顺序与文件头中**“SYS / # / OBS TYPES”**记录的观测类型顺序一致。
  


   - 3版本：

     ![image-20231028161613724](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028161613724.png)
     
     - 每历元数据用用**>**开头 
     - 2~29位为历元时间（yyyy mm dd hh mm ss)
     - 32位为 epoch flag
     - 后面是接收机钟差（s）
     - 每组数据中的每一行表示一颗卫星的观测值，观测值的顺序与文件头中**“SYS / # / OBS TYPES”**记录的观测类型顺序一致 
     
     



**执行流程**：

* 2版本：
  * 读取卫星数到 `n` 
  * 读取`epoh flag` 到 `flag`
  * 读取历元时间 `time`
  * 循环读取卫星ID(G10、G32、G26) ，读到68列，还没把卫星读完，就fgets()读取新的一行 
  * 将卫星ID转为`satellite number`，存到`sats[]`数组中 
* 3版本：
  * 读取卫星数量 `n`
  * 读取`epoh flag` 到 `flag`
  * 读取历元时间 `time`



#### 5. decode_obsdata()：读取一个历元内一颗卫星的观测值



* 3版本，读取卫星`satellite number`存到`obs->sat `
* 星系统和`mask`做与运算，判断卫星系统是否启用 
* 根据卫星系统分配索引 `ind` 
* 根据索引`ind`中的观测值类型，循环读取观测值，读取一个历元内，一颗卫星的观测值，记录有效的观测值到 `val[i]` ，记录记录信号失锁到 `lli[i]`
* 初始化`obs`各观测值数组，赋空 
* 遍历观测值类型，同频率的观测码，下标分别存到`k[]`，`l[]`中 ，`p[]` 存频率索引，后面`obs->P[0]`就是利用L1载波观测到的伪距，`obs->P[1]`就是利用L2载波观测到的伪距 
* 同一个频率有不同的观测码，取优先级高的 
* 根据索引`ind`中的观测值类型，遍历观测值，`val[i]`、`lli[i]`存入`obs`中 





### 7、星历文件读取

#### 1. decode_navh()、decode_gnavh()、decode_hnavh()

以decode_navh()为例：

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/819ea1b1474442a09edd951a7d9409dd.png)



#### 2. readrnxnav()：读取星历文件，添加到nav结构体中



* **add_eph**()：nav->eph[] 中添加 eph 星历数据，nav->n 表示 eph 数量。
* **add_geph**()：nav->geph[] 中添加 GLONASS 星历数据，nav->ng 表示 geph 数量。
* **add_seph**()：nav->seph[] 中添加 SBAS 星历数据，nav->ns 表示 seph 数量。



#### 3. readrnxnavb()：读取一个历元的星历数据，添加到 eph 结构体中



![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/7c62093881b847b8943b0910e78b23b8.png)

* 调用`set_sysmask()`获取卫星系统掩码

* 循环读取一行行，记录TOC，读取到`data[]`，i记录读取的数据数量，读够数量调用`decode_eph()`等函数赋值给`eph_t`结构体 



### 8、钟差文件读取





## 五、卫星位置钟差计算

### 1、satposs_rtklib()

```c
gtime_t teph     I     (gpst) 用于选择星历的时刻 (gpst)
obsd_t *obs      I      OBS观测数据
int    n         I      OBS数
nav_t  *nav      I      NAV导航电文
int    ephopt    I      星历选项 (EPHOPT_???)
double *rs       O      卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double *dts      O      卫星钟差，长度为2*n， {bias,drift} (s|s/s)
double *var      O      卫星位置和钟差的协方差 (m^2)
int    *svh      O      卫星健康标志 (-1:correction not available)
```

![image-20230929100826545](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100826545.png)

遍历观测数据，找伪距观测值，除以光速得到信号传播时间，用数据接收时刻减去伪距信号传播时间得到信号发射时刻。

调用 `ephclk()` 函数，由广播星历计算出当前观测卫星与 GPS 时间的钟差 `dt` ,此时的钟差是没有考虑相对论效应和 TGD 的 ，`dt` 仅作为`satpos()`的参数，不作为最终计算的钟差。信号发射时刻减去钟差 `dt`，得到 GPS 时间下的卫星信号发射时刻。

**调用 `satpos()` 对此观测值进行下一步卫星位置钟差的计算**；`satpos()` 函数对星历计算选项进行判断，**广播星历模式调用 `ephpos()`**，**精密星历模式调用 `peph2pos()`**。最后检测钟差值，如果没有精密星历，则调用 `ephclk()` 用广播星历计算钟差。

```c
extern void satposs_rtklib(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
                    int ephopt, double *rs, double *dts, double *var, int *svh)
{
    gtime_t time[MAXOBS]={{0}};
    double dt,pr;
    int i,j;
    
    for (i=0;i<n&&i<2*MAXOBS;i++) {
        for (j=0;j<6;j++) rs [j+i*6]=0.0;
        for (j=0;j<2;j++) dts[j+i*2]=0.0;
        var[i]=0.0; svh[i]=0;
        
        /* search any psuedorange */
        for (j=0,pr=0.0;j<NFREQ;j++) if ((pr=obs[i].P[j])!=0.0) break;
        
        if (j>=NFREQ) {
            sprintf(PPP_Glo.chMsg,"*** WARNING: no pseudorange %s sat=%2d\n",
				time_str(obs[i].time,3),obs[i].sat);
			outDebug(OUTWIN,OUTFIL,0);
            continue;
        }
        /* transmission time by satellite clock */
        time[i]=timeadd(obs[i].time,-pr/CLIGHT);
        
        /* satellite clock bias by broadcast ephemeris */
        if (!ephclk(time[i],teph,obs[i].sat,nav,&dt)) {
            sprintf(PPP_Glo.chMsg,"*** WARNING: no broadcast clock %s sat=%2d\n",
				time_str(time[i],3),obs[i].sat);
			outDebug(0,OUTFIL,0);
            continue;
        }
        time[i]=timeadd(time[i],-dt);
        
        /* satellite position and clock at transmission time */
        if (!satpos(time[i],teph,obs[i].sat,ephopt,nav,rs+i*6,dts+i*2,var+i,
                    svh+i)) {
            sprintf(PPP_Glo.chMsg,"*** WARNING: no ephemeris %s sat=%2d\n",
				time_str(time[i],3),obs[i].sat);
			outDebug(0,0,0);
            continue;
        }
        /* if no precise clock available, use broadcast clock instead */
        if (dts[i*2]==0.0) {
            if (!ephclk(time[i],teph,obs[i].sat,nav,dts+i*2)) continue;
            dts[1+i*2]=0.0;
            *var=SQR(STD_BRDCCLK);
        }
    }
}
```

### 2、ephclk()

![image-20230929100921394](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100921394.png)

单观测值卫星钟差计算。由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2clk()` 根据广播星历参数 $a_0$、$a_1$、$a_2$ 计算卫星钟差（迭代 3 次）；

如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2clk()` 根据广播星历参数 $t_aun$、$g_aun$  计算卫星钟差（迭代 3 次）。

```c
static int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  double *dts)
{
    eph_t  *eph;
    geph_t *geph;
    int sys;
    
    sys=satsys(sat,NULL);
    
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
        if (!(eph=seleph(teph,sat,-1,nav))) return 0;
        *dts=eph2clk(time,eph);
    }
    else if (sys==SYS_GLO) {
        if (!(geph=selgeph(teph,sat,-1,nav))) return 0;
        *dts=geph2clk(time,geph);
    }
    else return 0;
    
    return 1;
}
```

### 3、ephpos()

![image-20230929101151404](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929101151404.png)

与 `ephclk()` 同理，由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2pos()` 根据广播星历中的开普勒轨道参数和摄动改正计算卫星位置（对北斗 MEO、IGSO 卫星会进行特殊处理）、校正卫星钟差的相对论效应、调用 `var_uraeph()` 用 URA 值来标定方差。

如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2pos()` 根据广播星历中 PZ-90 坐标系下卫星状态向量四阶龙格库塔迭代计算卫星位置。

计算完一次位置之后，加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟漂。

```c
static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  int iode, double *rs, double *dts, double *var, int *svh)
{
    eph_t  *eph;
    geph_t *geph;
    double rst[3],dtst[1],tt=1E-3;
    int i,sys;
    
    sys=satsys(sat,NULL);
    
    *svh=-1;
    
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
        if (!(eph=seleph(teph,sat,iode,nav))) return 0;
        eph2pos(time,eph,rs,dts,var);
        time=timeadd(time,tt);
        eph2pos(time,eph,rst,dtst,var);
        *svh=eph->svh;
    }
    else if (sys==SYS_GLO) {
        if (!(geph=selgeph(teph,sat,iode,nav))) return 0;
        geph2pos(time,geph,rs,dts,var);
        time=timeadd(time,tt);
        geph2pos(time,geph,rst,dtst,var);
        *svh=geph->svh;
    }
    else return 0;
    
    /* satellite velocity and clock drift by differential approx */
    for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;
    dts[1]=(dtst[0]-dts[0])/tt;
    
    return 1;
}
```

### 4、peph2pos()

![image-20230929101206005](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929101206005.png)

调用 `pephpos()` 根据精密星历计算卫星位置钟差，其中先二分查找时间最接近的精密星历，然后地球自转改正，调用 `interppol()` 内维尔插值获取卫星位置、线性插值获取钟差，最后计算标准差。

调用 `pephclk()` 根据精密星历计算卫星位置钟差，其中先二分查找时间最接近的精密钟差，再线性插值获取钟差、计算标准差。

计算相对论效应改正量，调用 `satantoff()` 计算卫星天线相位偏差改正。加上改正量得到卫星位置钟差。

加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟飘。

```c
extern int peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
                    double *rs, double *dts, double *var)
{
    double rss[3],rst[3],dtss[1],dtst[1],dant[3]={0},vare=0.0,varc=0.0,tt=1E-3;
    int i;
    
    if (sat<=0||MAXSAT<sat) return 0;
    
    /* satellite position and clock bias */
    if (!pephpos(time,sat,nav,rss,dtss,&vare,&varc)||
        !pephclk(time,sat,nav,dtss,&varc)) return 0;
    
    time=timeadd(time,tt);
    if (!pephpos(time,sat,nav,rst,dtst,NULL,NULL)||
        !pephclk(time,sat,nav,dtst,NULL)) return 0;

    /* satellite antenna offset correction */
    if (opt) {
        satantoff(time,rss,sat,nav,dant);
    }

    for (i=0;i<3;i++) {
        rs[i  ]=rss[i]+dant[i];
        rs[i+3]=(rst[i]-rss[i])/tt;
    }
    /* relativistic effect correction */
    if (dtss[0]!=0.0) {
        dts[0]=dtss[0]-2.0*dot(rs,rs+3,3)/CLIGHT/CLIGHT;
        dts[1]=(dtst[0]-dtss[0])/tt;
    }
    else    /* no precise clock */
        dts[0]=dts[1]=0.0;
    
    *var=vare+varc;

    return 1;
}
```





