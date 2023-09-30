>原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning
[TOC]

## 一、GAMP 简介

### 1、程序概述

GAMP 全称 (**G**NSS  **A**nalysis software for **M**ulti-constellation and multi-frequency **P**recise positioning)，在 RTKLIB 的基础上，将一些些多余的函数、代码简洁化，精简出后处理 PPP 部分，并对算法进行改进增强。由于做了简化，代码比 RTKLIB 原版还要简单，对初学者非常友好，在我接触过的导航定位开源程序中算是最简单的。使用也很方便，软件包里提供了 VS 工程，和组织好的配置文件、数据文件，简单改改路径就能算出结果。





### 2、资源获取







### 3、代码分析







### 4、函数调用关系

![1688082358362](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1688082358362.png)

### 5、程序执行流程图







## 二、VS2022下 编译调试调试









## 三、配置文件











## 四、主要的数据结构











## 五、文件读取、预处理

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

`proccfgfile()` 函数先将  `PPP_Glo` 结构体初始化，将处理方式，输入输出文件路径赋空值。打开传入的 `gamp.cfg` 文件。获取观测值文件路径和处理方式，根据观测文件的数量调用对应的函数，单个观测文件**调用 `procOneFile()` 进行下一步处理**；如果有多个文件调用 `batchProc()` 进行批量处理，`batchProc()` 会打开文件夹，循环查找文件夹中的观测值O文件，**调用 `procOneFile()` 进行下一步处理**。

> 观测值`O`文件的后缀有两种，一种是直接 `.O` 结尾，一种是 `.ddO` 结尾，dd表示年份后两位。

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

先调用 `preProc()` 预处理：通过调用 `initGlobal()` 初始化 `PPP_Glo` 结构体；调用 `getObsInfo()`  读取观测O文件的一部分，获取起止时间、文件版本、天线种类等基础信息；为 `filopt.inf`、`filopt.outf` 开辟内存空间。

调用 `readcfgFile()` 读取整个配置文件，通过 `strstr(line,"start_time")` 匹配处理选项，存储到 `prcOpt_Ex`、`prcopt`。

调用 `getFopt_auto()` ，通过调用 `findClkFile()`、`findNavFile()`，根据后缀名自动查找各种 PPP 解算所需的文件，将文件路径存到 `fopt->inf` 中。

**调用 `gampPos()` 进行下一步处理**；处理结束，调用 `postProc()` 释放 `filopt.inf`、`filopt.outf` 内存空间。

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

先调用 `outhead()` 写输出文件的文件头。调用 `setcodepri()` 设置观测值优先级。调用 `readdcb()`、`readobsnav()`、`readpreceph()` 等函数读取文件。

文件读取完之后，**调用 `execses()` 进行下一步处理**。处理完之后调用 `freeobsnav()`、`freepreceph()` 释放内存空间。

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

先调用 `sampledetermine()` 获取观测值采用间隔（解算频率），调用 `calCsThres()` 获取周跳检测的阈值，调用 `rtkinit()` 初始化 `rtk` 结构体。**调用 `procpos()` 进行下一步处理**。处理完之后调用 `rtkfree()` 释放 `rtk` 结构体。

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

循环调用 `inputobs()` 传入一个历元的观测值。调用 `obsScan_SPP()` 观测值检测、调用 `BDmultipathCorr()` 修正北斗伪距。**调用 `rtkpos()` 进行下一步处理**。处理完之后调用 `outResult()`、`outsol()` 输出结果。

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

至此已经读完了文件，开始进行逐历元解算，先**调用 `spp()` 进行 SPP 解算**，调用 `obsScan_PPP()` 观测值检测，调用 `clkRepair()` 修复钟跳，**调用 `pppos()` 进行 PPP 解算**，调用 `calDop()` 计算各种 DOP 值，调用 `keepEpInfo()` 存储当前历元的信息，其中会调用 `gfmeas()`、`wlAmbMeas()`。

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

## 六、卫星位置钟差计算

### 1、satposs_rtklib()

![image-20230929100826545](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100826545.png)

遍历观测数据，找伪距观测值，除以光速得到信号传播时间，用数据接收时刻减去伪距信号传播时间得到信号发射时刻。

调用 `ephclk()` 函数，由广播星历计算出当前观测卫星与 GPS 时间的钟差 `dt` ,此时的钟差是没有考虑相对论效应和 TGD 的 ，`dt` 仅作为`satpos()`的参数，不作为最终计算的钟差。信号发射时刻减去钟差 `dt`，得到 GPS 时间下的卫星信号发射时刻。

**调用 `satpos()` 对此观测值进行下一步卫星位置钟差的计算**；`satpos()` 函数对星历计算选项进行判断，**广播星历模式调用 `ephpos()`**，**精密星历模式调用 `peph2pos()`**。最后检测钟差值，如果没有精密星历，则调用 `ephclk()` 用广播星历计算钟差。

### 2、ephclk()

![image-20230929100921394](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100921394.png)

单观测值卫星钟差计算。由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2clk()` 根据广播星历参数 $a_0$、$a_1$、$a_2$ 计算卫星钟差（迭代 3 次）；

如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2clk()` 根据广播星历参数 $t_aun$、$g_aun$  计算卫星钟差（迭代 3 次）。



### 3、ephpos()

![image-20230929101151404](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929101151404.png)

与 `ephclk()` 同理，由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2pos()` 根据广播星历中的开普勒轨道参数和摄动改正计算卫星位置（对北斗 MEO、IGSO 卫星会进行特殊处理）、校正卫星钟差的相对论效应、调用 `var_uraeph()` 用 URA 值来标定方差。

如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2pos()` 根据广播星历中 PZ-90 坐标系下卫星状态向量四阶龙格库塔迭代计算卫星位置。

计算完一次位置之后，加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟漂。



### 4、peph2pos()

![image-20230929101206005](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929101206005.png)

调用 `pephpos()` 根据精密星历计算卫星位置钟差，其中先二分查找时间最接近的精密星历，然后地球自转改正，调用 `interppol()` 内维尔插值获取卫星位置、线性插值获取钟差，最后计算标准差。

调用 `pephclk()` 根据精密星历计算卫星位置钟差，其中先二分查找时间最接近的精密钟差，再线性插值获取钟差、计算标准差。

计算相对论效应改正量，调用 `satantoff()` 计算卫星天线相位偏差改正。加上改正量得到卫星位置钟差。

加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度中飘。



## 七、SPP 解算

![image-20230929100318941](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100318941.png)

### 1、spp()

默认使用广播星历计算卫星位置、钟差，使用克罗布歇模型通过广播星历中的参数计算电离层延迟，使用 Saastamoinen 模型计算对流层延迟。**调用 `satposs_rtklib()` 计算卫星位置、卫星钟差**，**调用 `estpos()` 计算接收机位置。**











## 八、PPP 解算

![image-20230929100630354](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100630354.png)























