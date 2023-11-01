>原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

![1698494049(1)](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1698494049(1).png)

[TOC]

## 一、GAMP 简介

### 1、程序概述

GAMP 全称 (**G**NSS  **A**nalysis software for **M**ulti-constellation and multi-frequency **P**recise positioning)，在 RTKLIB 的基础上，将一些多余的函数、代码简洁化，精简出后处理 PPP 部分，并对算法进行改进增强。简化后代码比 RTKLIB 原版还要简单，对初学者非常友好，在我接触过的导航定位开源程序中算是最简单的。使用也很方便，软件包里提供了 VS 工程，和组织好的配置文件、数据文件；设置好 pthreads 库，简单改改文件路径就能算出结果。

![GAMP 系统结构图](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GAMP%20%E7%B3%BB%E7%BB%9F%E7%BB%93%E6%9E%84%E5%9B%BE.png)

### 2、相较于 RTKLIB 的增强

* **非差非组合、UOFC 的 PPP 模型**
* **钟跳修复**
* **北斗多路径延迟改正**
* **观测值信号量支持更多**
* **抗差估计**
* **伪距观测值质量检测**
* **MW + GF 周跳检测**
* **利用残差粗差探测**
* **计算了更多的 DOP 值**
* **对流层 GPT 模型**
* **GLONASS 伪距 IFB** 
* **输出结果更多**
* **GPT对流层模型**

### 3、函数调用关系

![1688082358362](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1688082358362.png)

### 4、程序执行流程





### 5、配置文件

根据配置文件的注释，在示例配置文件基础上修改即可。

* **obs file/folder**：观测文件/文件夹路径；如果填 0 需要指定观测文件绝对路径，如果填 1 需要指定包含所有解算用的文件的文件绝对路径。
* **start_time、end_time**：解算起止时间，填 0 表示全部解算。
* **posmode**：定位模式，支持三种：SPP、static  PPP、kinematic PPP。
* **soltype**：滤波模式：0 正向、1 反向、2 正方向结合
* **navsys**：卫星系统，用 7 为二进制码表示：1 GPS、4 GLONASS、8 Galileo、16 QZSS、32 BDS；比如使用 BDS、GPS 要填 33。
* **gnsisb**：ISB 噪声模型：1 常值、2 精确先验常值、3 随机游走、4 白噪声。
* **gloicb**：GLONASS 伪距 IFB模型：0 忽略、1 线性模型、2 二次多项式模型、3 估计每颗卫星、3 估计每个频率。
* **minelev**：卫星截止高度角，默认 10。
* **maxout**：重置模糊度的最低停测段数，默认 3。
* **sampleprc**：截止观测值，默认 0。
* **inpfrq**：频率选择：1 单频或者消电离层 PPP、2 双频非差非组合 PPP
* **ionoopt**：电离层处理选项：0 不处理、1 广播星历克罗布歇模型、2 消电离层、3、单频估计、4 双频估计、5 电离层格网模型。
* **ionopnoise**：估计电离层随机随机：1 静态、2 随机游走、3 白噪声。
* **ionoconstraint**：增加电离层的虚拟观测参数及其对观测方程的相应约束：0 关闭、1 打开。
* **troopt**：对流层处理选项：0 不处理、1 Saastamoninen 模型、2 SBAS 模型、3 估计 ZTD、估计 ZTD 格网。
* **tropmf**：对流层投影映射函数：0 NMF、1 GMF。
* **tidecorr**：潮汐改正模式，与卫星系统类似用 3 位二进制码表示：1 固体潮、2 海洋潮、3 极潮，如果想全部改正就设置为 7。
* **cycleslip_GF**：是否开启 GF 周跳检测：0 关闭、1 打开。
* **cycleslip_MW**：是否开启 MW 周跳检测：0 关闭、1 打开。
* **errratio(P/L GPS)、errratio(P/L GLO)、errratio(P/L BDS)、errratio(P/L GAL)、errratio(P/L QZS)**：伪距和载波位观测值误差的比率，默认100。
* **errmeas(L)**：载波相位观测值精度，默认 0.003 m。
* **prcNoise(AMB)**：估计模糊度参数的观测噪声。
* **prcNoise(ZTD)**：估计 ZTD 参数的观测噪声。
* **prcNoise(ION)**：估计 ION 参数的观测噪声。
* **prcNoise(ION_GF)**：估计 ION_GF 参数的观测噪声。
* **outdir**：结果输出文件夹，当前工作目录的子文件夹。
* **output**：设置输出哪些结果，0 不输出、1 输出，包括：
  * pos、debug、pdop、elev、dtrp、ifamb、wlamb_no、wlamb_yes、gf、amb_cs、resc1、resc2、resc3、resp1、resp2、resp3、stec、isb、ibm、ifb、ippp

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

* 先调用 `outhead()` 输出结果文件的文件头。
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

#### 1. setcodepri()：设置信号优先级。

如果输入的观测数据在同一频率内包含多个信号，GAMP 将按照以下默认优先级选择一个信号进行处理。 

```c
static char codepris[7][MAXFREQ][16]={  /* code priority table */
   
   /* L1/E1      L2/B1        L5/E5a/L3 L6/LEX/B3 E5b/B2    E5(a+b)  S */
    {"CPYWMNSL","PYWCMNDSLX","IQX"     ,""       ,""       ,""      ,""    }, /* GPS */
    {"PC"      ,"PC"        ,"IQX"     ,""       ,""       ,""      ,""    }, /* GLO */
    {"CABXZ"   ,""          ,"IQX"     ,"ABCXZ"  ,"IQX"    ,"IQX"   ,""    }, /* GAL */
    {"CSLXZ"   ,"SLX"       ,"IQX"     ,"SLX"    ,""       ,""      ,""    }, /* QZS */
    {"C"       ,""          ,"IQX"     ,""       ,""       ,""      ,""    }, /* SBS */
    {"IQX"     ,"IQX"       ,"IQX"     ,"IQX"    ,"IQX"    ,""      ,""    }, /* BDS */
    {""        ,""          ,"ABCX"    ,""       ,""       ,""      ,"ABCX"}  /* IRN */
};
```

 用 `setcodepri()`，可改变优先级顺序：

```c
extern void setcodepri(int sys, int freq, const char *pri)
{   
    if (freq<=0||MAXFREQ<freq) return;
    if (sys&SYS_GPS) strcpy(codepris[0][freq-1],pri);
    if (sys&SYS_GLO) strcpy(codepris[1][freq-1],pri);
    if (sys&SYS_GAL) strcpy(codepris[2][freq-1],pri);
    if (sys&SYS_QZS) strcpy(codepris[3][freq-1],pri);
    if (sys&SYS_SBS) strcpy(codepris[4][freq-1],pri);
    if (sys&SYS_CMP) strcpy(codepris[5][freq-1],pri);
    if (sys&SYS_IRN) strcpy(codepris[6][freq-1],pri);
}
```

#### 2. outhead()：输出结果文件头

代码都被注释了，就只是创建文件，需要输出得手动改这个文件：

```c
static int outhead(char **outfile, const prcopt_t *popt, const solopt_t *sopt, FILE* fp[], int sum)
{
	int i;

	for (i=0; i<sum; i++) {
		if (!outfile[i]||strlen(outfile[i])<=4) continue;

		createdir((const char *)outfile[i]);

		if (!(fp[i]=fopen(outfile[i],"w"))) {
			printf("error : open output file %s",outfile[i]);
			return 0;
		}

		switch (i) {
		case 0:
			//outsolhead(fp[0],sopt);
			break;
		case 1:
			//outCsInfoHead(fp[1],sopt);
			break;
		case 2:
			//outResiHead(fp[2],popt,sopt);
			break;
		case 3:
			//outResiHead(fp[3],popt,sopt);
			break;
		case 4:
			//outResiHead(fp[4],popt,sopt);
			break;
		case 5:
			//outAllAmbHead(fp[5],sopt,12);
			break;
		case 6:
			//outAllAmbHead(fp[6],sopt,7);
			break;
		case 7:
			//outAllAmbHead(fp[7],sopt,7);
			break;
		case 8:
			//outAllAmbHead(fp[8],sopt,7);
			break;
		case 9:
			//outAllAmbHead(fp[9],sopt,7);
			break;
		//case 10:
		//    outallambhead(fp[10],sopt,7);
		//    break;
		//case 11:
		//    outallambhead(fp[11],sopt,12);
		//    break;
		}
		fclose(fp[i]);
	}

	return 1;
}
```

### 5、excses()：执行后处理解算

![image-20230929095710372](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929095710372.png)

* 先调用 `sampledetermine()` 获取观测值采用间隔（解算频率）。
* 调用 `calCsThres()` 获取根据采样频率周跳检测的阈值，调用 `rtkinit()` 初始化 `rtk` 结构体。
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

#### 1. sampledetermine()：确定采样间隔

* 观测值少于 MINNUM，直接返回 30。

* 获取观测值时间序列 `gt`。
* 遍历 MINNUM 个数据，获取采样间隔序列 `dt`、和这个采样间隔出现的次数序列 `it`。
* 找出出现次数最多的采样间隔下标 `m`，返回出现次数最多的采样间隔 `dt[m]`。

```c
static double sampledetermine(const prcopt_t *popt)
{
	obsd_t obs[MAXOBS];
	int i,j,nobs,n,m,it[MINNUM];
	gtime_t gt[MINNUM+1];
	double dt[MINNUM],dtmp;

	j=0;
	PPP_Glo.iObsu=PPP_Glo.revs=PPP_Glo.iEpoch=0;

	// 获取观测值时间序列 gt
	//MINNUM=30
	while ((nobs=inputobs(obs,obss,PPP_Glo.revs,&PPP_Glo.iObsu,&PPP_Glo.iEpoch))>=0) {
		gt[j++]=obs[0].time;

		if (j>MINNUM) break;
	}

	PPP_Glo.iObsu=PPP_Glo.revs=PPP_Glo.iEpoch=0;

	// 观测值少于 MINNUM，直接返回 30
	if (j<=MINNUM) {
		sprintf(PPP_Glo.chMsg,"*** WARNING: the number of epochs is less than %d.\n",MINNUM);
		outDebug(OUTWIN,OUTFIL,0);

		return 30.0;
	}

	for (i=0;i<MINNUM;i++) {
		dt[i]=0.0;
		it[i]=0;
	}

	// 求前两个数据时间差
	dt[0]=timediff(gt[1],gt[0]);
	it[0]=1;
	n=1;

	// 遍历 MINNUM 个数据，获取采样间隔序列 dt、和这个采样间隔出现的次数序列 it
	for (i=0;i<MINNUM;i++) {
		dtmp=timediff(gt[i+1],gt[i]);

		// 寻找与这个采样间隔接近的值，如果找到了，就增加对应的 it 的值；
		// 如果没有找到，就在 dt 中添加这个采样间隔，并将对应的 it 设为1
		for (j=0;j<n;j++) {
			if (fabs(dtmp-dt[j])<1.0e-8) {
				it[j]++;
				break;
			}
		}
		if (j==n) {
			dt[n]=dtmp;
			it[n++]=1;
		}
	}

	// 找出出现次数最多的采样间隔下标 m 
	for (i=j=m=0;i<MINNUM;i++) {
		if (j<it[i]) {
			j=it[i];
			m=i;
		}
	}

	if (3*j>=MINNUM) {
		return dt[m];
	}
	else {
		for (i=0;i<n;i++) {
			sprintf(PPP_Glo.chMsg,"Sample is %6.2f\n",dt[i]);
			outDebug(OUTWIN,0,0);
		}
		strcpy(PPP_Glo.chMsg,"*** ERROR: sampling may be inaccurate!\n");
		outDebug(OUTWIN,0,0);
	}

	// 返回出现次数最多的采样间隔 dt[m]
	return dt[m];
}
```

#### 2. calCsThres()：计算 MF、GF 周跳检测阈值

联合使用 Geometry-free (GF) 和 MW 组合观测值进行非差周跳探测，其充分利用了双频观测值线性组合的特点。GF 和 MW 组合观测值分别为：
$$
\begin{array}{c}
L_{\mathrm{GF}}(i)=\lambda_{1} \Phi_{1}(i)-\lambda_{2} \Phi_{2}(i)=\left(\gamma_{2}-1\right) I_{1}(i)+\left(\lambda_{1} N_{1}-\lambda_{2} N_{2}\right) \\
\left\{\begin{array}{l}
\lambda_{\delta} \Phi_{\delta}(i)=\left(f_{1} \lambda_{1} \Phi_{1}(i)-f_{2} \lambda_{2} \Phi_{2}(i)\right) /\left(f_{1}-f_{2}\right)=\rho(i)+f_{1} f_{2} /\left(f_{1}^{2}-f_{2}^{2}\right) \cdot I_{1}(i)+\lambda_{\delta} N_{\delta} \\
P_{\delta}(i)=\left(f_{1} P_{1}(i)+f_{2} P_{2}(i)\right) /\left(f_{1}+f_{2}\right)=\rho(i)+f_{1} f_{2} /\left(f_{1}^{2}-f_{2}^{2}\right) \cdot I_{1}(i) \\
N_{\delta}=N_{1}-N_{2}=\Phi_{\delta}(i)-P_{\delta}(i) / \lambda_{\delta} \\
\lambda_{\delta}=c /\left(f_{1}-f_{2}\right)
\end{array}\right.
\end{array}
$$
式中，$i$ 表示观测历元号; $\lambda_{\delta}$ 和 $N_{\delta}$ 分别为宽巷波长和宽巷模糊度。 可以看出，MW 组合的精度受伪距观测噪声和多路径效应的影响，可通过下述递推公式减弱其影响，第 $i$ 个历元的 MW 组合观测量的平均值及方差为：
$$
\begin{array}{c}
\left\langle N_{\delta}\right\rangle_{i}=\left\langle N_{\delta}\right\rangle_{i-1}+\frac{1}{i}\left(N_{\delta i}-\left\langle N_{\delta}\right\rangle_{i-1}\right) \\
\sigma_{i}^{2}=\sigma_{i-1}^{2}+\frac{1}{i}\left\{\left(N_{\delta i}-\left\langle N_{\delta}\right\rangle_{i-1}\right)^{2}-\sigma_{i-1}^{2}\right\}
\end{array}
$$
式中, 〈〉表示多个历元的平滑值。对于 GF 组合，利用当前历元组合观测值与前一历元组合观测值的差值的绝对值 $\left|L_{\mathrm{GF}}(i)-L_{\mathrm{GF}}(i-1)\right|$ 作为检验量进行周跳探测。对于 MW 组合，将当前历元 $i$ 的 MW 观测量 $N_{\delta i}$ 与前 $i-1$ 历元宽巷模糊度平滑值 $\left\langle N_{\delta}\right\rangle_{i-1}$ 差值的绝对值进行比较判断是否发生周跳。顾及观测数据的采样率和高度角，给出确定周跳探测经验阈值：
$$
\begin{array}{l}\begin{array}{l}R_{\mathrm{GF}}(E, R)=\left\{\begin{array}{cc}(-1.0 / 15.0 \cdot E+2) \cdot b_{\mathrm{GF}}, & E \leq 15^{\circ} \\ b_{\mathrm{GF}}, & E>15^{\circ}\end{array}\right. \\ b_{\mathrm{GF}}(R)=\left\{\begin{array}{cc}0.05 \mathrm{~m}, & 0<R \leq 1 \mathrm{~s} \\ 0.1 / 20.0 \cdot R+0.05 \mathrm{~m}, & 1<R \leq 20 \mathrm{~s} \\ 0.15 \mathrm{~m}, & 20<R \leq 60 \mathrm{~s} \\ 0.25 \mathrm{~m}, & 60<R \leq 100 \mathrm{~s} \\ 0.35 \mathrm{~m}, & \text { 其它 }\end{array}\right.\end{array} \\ R_{\mathrm{MW}}(E, R)=\left\{\begin{array}{cc}(-0.1 \cdot E+3) \cdot b_{\mathrm{MW}}, & E \leq 20^{\circ} \\ b_{\mathrm{MW}}, & E>20^{\circ}\end{array}\right. \\ b_{\text {MW }}(R)=\left\{\begin{array}{cc}2.5 \mathrm{c}, & 0<R \leq 1 \mathrm{~s} \\ 2.5 / 20.0 \cdot R+2.5 \mathrm{c}, & 1<R \leq 20 \mathrm{~s} \\ 5.0 \mathrm{c}, & 20<R \leq 60 \mathrm{~s} \\ 7.5 \mathrm{c}, & \text { 其它 }\end{array}\right. \\\end{array}
$$
式中，$R_{\mathrm{GF}}$ (单位: $\mathrm{m}$ 或米) 和 $R_{\mathrm{MW}}$ (单位: $\mathrm{c}$ 或周) 分别为 $\mathrm{GF}$ 组合和 $\mathrm{MW}$ 组合周跳检验量的阈值；$E 、 R$ 分别为卫星高度角 (单位：度) 和观测值采样间隔 (单位：$s$ )。

```c
extern int calCsThres(prcopt_t *opt, const double sample)
{
	int b=0;

	if (sample>0.0) {
		if (PPP_Glo.prcOpt_Ex.bUsed_gfCs==1&&fabs(PPP_Glo.prcOpt_Ex.csThresGF)<0.01) {
			if (sample<=1.0)        PPP_Glo.prcOpt_Ex.csThresGF=0.05;
			else if (sample<=20.0)  PPP_Glo.prcOpt_Ex.csThresGF=(0.10)/(20.0-0.0)*sample+0.05;
			else if (sample<=60.0)  PPP_Glo.prcOpt_Ex.csThresGF=0.15;
			else if (sample<=100.0) PPP_Glo.prcOpt_Ex.csThresGF=0.25;
			else                    PPP_Glo.prcOpt_Ex.csThresGF=0.35;

			b=1;
		}
		if (PPP_Glo.prcOpt_Ex.bUsed_mwCs==1&&fabs(PPP_Glo.prcOpt_Ex.csThresMW)<0.01) {
			if (sample<=1.0)        PPP_Glo.prcOpt_Ex.csThresMW=2.5;
			else if (sample<=20.0)  PPP_Glo.prcOpt_Ex.csThresMW=(2.5)/(20.0-0.0)*sample+2.5;
			else if (sample<=60.0)  PPP_Glo.prcOpt_Ex.csThresMW=5.0;
			else                    PPP_Glo.prcOpt_Ex.csThresMW=7.5;

			b=1;
		}

		return b;
	}
	else {
		//sample<=0.0
		PPP_Glo.prcOpt_Ex.csThresGF=0.15;
		PPP_Glo.prcOpt_Ex.csThresMW=5.0;
		b=0;
	}

	return b;
}
```

### 6、procpos()：进行定位解算

![image-20230929095813550](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929095813550.png)

* 循环调用 `inputobs()` 传入一个历元的观测值。
* 调用 `obsScan_SPP()`筛选出能进行 SPP 解算的观测值。
* 调用 `BDmultipathCorr()` 修正北斗伪距延迟。
* 调用 `rtkpos()` 进行下一步处理。
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

#### 1. BDmultipathCorr()：北斗伪距偏差改正

研究发现，BDS2 卫星存在一种可能由多路径引起的卫星端的伪距观测值系统偏差，称为北斗卫星伪距偏差，也称北斗伪距多路径延迟。该伪距偏差只存在于伪距观测值，和卫星高度角密切相关，可基于观测值的 *MP* 组合得到：
$$
\begin{aligned} M P_{m} & =P_{m}-\frac{f_{m}^{2}+f_{n}^{2}}{f_{m}^{2}-f_{n}^{2}} \cdot L_{m} \cdot \lambda_{m}+\frac{2 f_{n}^{2}}{f_{m}^{2}-f_{n}^{2}} \cdot L_{n} \cdot \lambda_{n} \\ & =M P_{m}-\frac{f_{m}^{2}+f_{n}^{2}}{f_{m}^{2}-f_{n}^{2}} \cdot M_{L_{m}}+\frac{2 f_{n}^{2}}{f_{m}^{2}-f_{n}^{2}} \cdot M_{L_{n}}+B_{m}+\varepsilon\end{aligned}
$$
式中，$m, n(m \neq n)$ 表示频率编号；$M P_{m}, M_{L_{m}}, M_{L_{n}}$ 分别表示频率 $m$ 的伪距多路径和频率 $m, n$ 的载波相位多路径；$B_{m}$ 包括载波相位模糊度和硬件延迟等, 在连续观测无周跳情况下为常数。*MP* 组合是无电离层无几何组合观测值，对其平滑后可分离出伪距多路径噪声。Wanninger 基于卫星高度角，采用分段线性拟合的方法建立了节点间隔 10 度的北斗二号 IGSO 和 MEO 卫星伪距偏差的经验模型，如下表所示，本文使用该模型，通过插值对 BDS2 伪距偏差进行改正：

![image-20231029145050005](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231029145050005.png)

代码中，先定义了基于卫星高度角，分段线性拟合模型 `IGSOCOEF`、`MEOCOEF`；然后遍历观测值，筛选出需要改正的北斗卫星，计算角度制的高度角 `elev`，取高度角的十位数 `b`，作为从 `IGSOCOEF`、`MEOCOEF` 中取值的下标；插值计算改正量 `dmp` 加到伪距上。

```c
extern void BDmultipathCorr(rtk_t *rtk, obsd_t *obs, int n)
{
	int i,j,sat,prn,b;
	double dmp[3],elev,a;

	// 基于卫星高度角，分段线性拟合模型
	const static double IGSOCOEF[3][10]={		/* m */
		{-0.55,-0.40,-0.34,-0.23,-0.15,-0.04,0.09,0.19,0.27,0.35},	//B1
		{-0.71,-0.36,-0.33,-0.19,-0.14,-0.03,0.08,0.17,0.24,0.33},	//B2
		{-0.27,-0.23,-0.21,-0.15,-0.11,-0.04,0.05,0.14,0.19,0.32},	//B3
	};
	const static double MEOCOEF[3][10]={		/* m */
		{-0.47,-0.38,-0.32,-0.23,-0.11,0.06,0.34,0.69,0.97,1.05},	//B1
		{-0.40,-0.31,-0.26,-0.18,-0.06,0.09,0.28,0.48,0.64,0.69},	//B2
		{-0.22,-0.15,-0.13,-0.10,-0.04,0.05,0.14,0.27,0.36,0.47},	//B3
	};

	// 遍历观测值
	for (i=0;i<n&&i<MAXOBS;i++) {
		sat=obs[i].sat;

		// 筛选出北斗卫星
        if (PPP_Glo.sFlag[sat-1].sys!=SYS_CMP) continue;

		// 剔除北斗一号卫星
		prn=PPP_Glo.sFlag[sat-1].prn;
		if (prn<=5) continue;

		// 计算角度制的高度角 elev
		elev=rtk->ssat[sat-1].azel[1]*R2D;
		
		if (elev<=0.0) continue;

		for (j=0;j<3;j++) dmp[j]=0.0;

		// 取高度角的十位数 b，作为从 IGSOCOEF、MEOCOEF 中取值的下标
		a=elev*0.1;
		b=(int)a;

		// 插值计算改正量 dmp
		if (prn>=6&&prn<11) { // IGSO(C06, C07, C08, C09, C10)
			if (b<0) {
				for (j=0;j<3;j++) dmp[j]=IGSOCOEF[j][0];
			}
			else if (b>=9) {
				for (j=0;j<3;j++) dmp[j]=IGSOCOEF[j][9];
			}
			else {
				for (j=0;j<3;j++) dmp[j]=IGSOCOEF[j][b]*(1.0-a+b)+IGSOCOEF[j][b+1]*(a-b);
			}
		}
		else if (prn>=11) {   // MEO(C11, C12, C13, C14)
			if (b<0) {
				for (j=0;j<3;j++) dmp[j]=MEOCOEF[j][0];
			}
			else if (b>=9) {
				for (j=0;j<3;j++) dmp[j]=MEOCOEF[j][9];
			}
			else {
				for (j=0;j<3;j++) dmp[j]=MEOCOEF[j][b]*(1.0-a+b)+MEOCOEF[j][b+1]*(a-b);
			}
		}

		// 伪距加上改正量 dmp
		for (j=0;j<3;j++) obs[i].P[j]+=dmp[j];
	}
}
```

#### 2. obsScan_SPP()：筛选出能进行 SPP 解算的观测值

遍历传入观测值列表 `obs`，根据启用的卫星系统 `popt->navsys`，排除的卫星 `popt->exsats` 筛选观测值，然后判断有没有伪距观测值，原路返回的 `obs` 就是筛选过后的观测值列表。

```c
extern void obsScan_SPP(const prcopt_t *popt, obsd_t *obs, const int nobs, int *nValid)
{
	double dt;
	int i,j,n,sat,sys;

	// 遍历观测值列表
	for (i=n=0;i<nobs;i++) {
		sat=obs[i].sat;
		sys=PPP_Glo.sFlag[sat-1].sys;

		// 根据启用的卫星系统 popt->navsys，排除的卫星 popt->exsats 筛选观测值
		/* exclude satellites */
		if (!(sys&popt->navsys)) continue;
		if (popt->exsats[sat-1])	 continue;

		// 判断有没有伪距观测值
		dt=0.0;
		for (j=0;j<NFREQ;j++) {
			dt+=obs[i].P[j]*obs[i].P[j];
		}
		if (dt==0.0)	 continue;

		// 返回的 obs 就是筛选过后的观测值列表
		obs[n++]=obs[i];
	}

	// 有效观测值数
	if (nValid) *nValid=n;
}
```

### 7、rtkpos()：逐历元解算

![image-20230929100123058](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100123058.png)

* 至此已经读完了文件，开始进行逐历元解算，先调用 `spp()` 进行 SPP 解算。
* 调用 `obsScan_PPP()` 筛选出能进行 PPP 解算的观测值。
* 调用 `clkRepair()` 修复钟跳。
* 调用 `pppos()` 进行 PPP 解算。
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

#### 1. clkRepair()：钟跳修复

一旦接收机发生钟跳，将破坏 GNSS 时标、伪距和载波相位观测值之间的一致性。根据钟跳对这三个基本量的影响方式，可将接收机钟跳分为四类，其定义与分类标准见下表：

![image-20231029152321938](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231029152321938.png)

其中，第二类和第三类钟跳会影响 MW 组合探测周跳的准确性，使其对周跳的探测失效。因此，关于接收机钟跳探测与修复的对象均是针对第二类和第三类钟跳（伪距观测值阶跃、载波相位观测值连续），采用观测值的历元间差分法进行实时钟跳探测与修复。令：
$$
\left\{\begin{array}{l}\Delta P^{s}(i)=P^{s}(i)-P^{s}(i-1) \\ \Delta L^{s}(i)=L^{s}(i)-L^{s}(i-1)\end{array}\right.
$$
式中， *P* 和 *L* 分别为原始的伪距和载波相位观测值。构造检验量 *T* 及其条件式：
$$
\left\{\begin{array}{l}T^{s}(i)=\Delta P^{s}(i)-\Delta L^{s}(i) \\ \left|T^{s}(i)\right|>k_{1} \approx 0.001 \cdot c\end{array}\right.
$$
式中，$k_{1}$ 为阈值。对于某一历元，当且仅当所有卫星满足上式中条件式时，才可以认为该历元时刻可能存在钟跳或所有卫星发生大周跳，此时利用下式计算钟跳候选值 $\varsigma$ 进而确定实际钟跳值$J(\mathrm{~ms})$ ：
$$
\begin{array}{c}
\varsigma=\alpha \cdot\left(\sum_{s=1}^{m} T^{s}\right) /(m \cdot c) \\
J=\left\{\begin{array}{cc}
\operatorname{int}(\varsigma), & |\varsigma-\operatorname{int}(\varsigma)| \leq k_{2} \\
0, & \text { 其它 }
\end{array}\right.
\end{array}
$$
式中，$\alpha$ 为系数因子，取 $\alpha=10^{3} ; k_{2}$ 为阈值，取 $k_{2}=10^{-5} \sim 10^{-7}$ 。

钟跳修复时，采用反向修复法，即当发生第二类或第三类钟跳时，将连续的载波相位观测值调整成阶跃形式，同伪距基准保持一致。其修复公式为：
$$
\tilde{L}^{s}(i)=L^{s}(i)+J \cdot c / \alpha
$$
式中，$\tilde{L}^{s}(i)$ 为修复后的载波相位观测值。

时标的跳变量的时间（以 ms 为单位）、伪距的跳变量是距离（以 m 为单位）、载波相位的跳变量是相位（以周为单位），要注意三者的转换。下面介绍代码：

* 先遍历传入的历元观测值列表 obs，没有双频伪距载波，直接跳过；
* 计算两时刻两时刻一二频率伪距载波变化量 d1、d2、d3、d4，
* 如果两时刻一频率伪距载波变化量的差值 d1-d3 超过了一毫秒的光程 290000，说明有毫秒级的钟跳。
* 平均一二频率平均跳变量 d1、d2（d2 没用到）。
* 存改正量到 `PPP_Glo.clkJump`

* 载波相位加上钟跳改正量 `ddd2/lam`

```cpp
static int clkRepair(obsd_t *obs, int n)
{
	int i,sat,validGps,cjGps;
	int bObserved[MAXPRNGPS];
	double delta0=0.0,delta1=0.0,d1,d2,d3,d4,ddd1,ddd2;
	double *lam;
	double CJ_F1,CJ_F2;

	for (i=0;i<MAXPRNGPS;i++) bObserved[i]=0;

	validGps=cjGps=0;

	// 遍历传入的历元观测值列表 obs
	for (i=0;i<n;i++) {
		sat=obs[i].sat;
		lam=PPP_Glo.lam[sat-1];

		if (sat>MAXPRNGPS) continue;

		// 没有双频伪距载波，直接跳过
		if (obs[i].P[0]*obs[i].P[1]*obs[i].L[0]*obs[i].L[1]==0.0) continue;

		if (PPP_Glo.obs0[sat-1][0]*PPP_Glo.obs0[sat-1][1]*PPP_Glo.obs0[sat-1][2]*PPP_Glo.obs0[sat-1][3]==0.0)
			continue;

		validGps++;

		d1=obs[i].P[0]-PPP_Glo.obs0[sat-1][0];			// 两时刻一频率伪距变化量
		d2=obs[i].P[1]-PPP_Glo.obs0[sat-1][1];			// 两时刻二频率伪距变化量
		d3=(obs[i].L[0]-PPP_Glo.obs0[sat-1][2])*lam[0];	// 两时刻一频率相位变化量
		d4=(obs[i].L[1]-PPP_Glo.obs0[sat-1][3])*lam[1];	// 两时刻二皮脸相位变化量

		// 290000 是一毫秒的光程，超过了说明有毫秒级的钟跳，
		if(fabs(d1-d3)>290000)   // ms clock jump
		{
			delta0+=d1-d3;
			delta1+=d2-d4;
			cjGps++;
		}
	}

	if(cjGps!=0&&cjGps==validGps)
	{	
		// 平均跳变量 d1、d2（d2 没用到）
		d1=delta0/cjGps;
		d2=delta1/cjGps;

		CJ_F1=0.0;   // flag for clock jump
		CJ_F2=0.0;
		CJ_F1=d1/CLIGHT*1000.0;
		CJ_F2=myRound(CJ_F1);

		if (fabs(CJ_F1-CJ_F2)<2.5E-2)
		{

			PPP_Glo.clkJump+=(int)CJ_F2;
			sprintf(PPP_Glo.chMsg, "*** WARNING: clock jump=%d(ms)\n",PPP_Glo.clkJump);
			outDebug(OUTWIN,OUTFIL,0);
		}
		else 
		{
			//
		}
	}

	// 
	for (i=0;i<n;i++) 
	{
		sat=obs[i].sat;

		if (sat>MAXPRNGPS) continue;

		bObserved[sat-1]=1;

		PPP_Glo.obs0[sat-1][0]=obs[i].P[0];
		PPP_Glo.obs0[sat-1][1]=obs[i].P[1];
		PPP_Glo.obs0[sat-1][2]=obs[i].L[0];
		PPP_Glo.obs0[sat-1][3]=obs[i].L[1];

		ddd1=PPP_Glo.clkJump*CLIGHT/1000.0;
		ddd2=PPP_Glo.clkJump*CLIGHT/1000.0;

		// 载波相位加上钟跳改正量
		//repair for phase observations
		if(obs[i].L[0]!=0.0) obs[i].L[0]+=ddd1/lam[0];

		if(obs[i].L[1]!=0.0) obs[i].L[1]+=ddd2/lam[1];

		//repair for code observations
		//if( obs[i].P[0]!=0.0 ) {
		//    obs[i].P[0]-=PPP_Glo.clkjump*CLIGHT/1000;
		//}
		//if( obs[i].P[1]!=0.0 ) {
		//    obs[i].P[1]-=PPP_Glo.clkjump*CLIGHT/1000;
		//}
	}

	for (i=0;i<MAXPRNGPS;i++) {
		if (bObserved[i]==0)
			PPP_Glo.obs0[i][0]=PPP_Glo.obs0[i][1]=PPP_Glo.obs0[i][2]=PPP_Glo.obs0[i][3]=0.0;
	}

	return 1;
}
```

#### 2. obsScan_PPP()：筛选出能进行 PPP 解算的观测值

遍历传入的观测值列表，剔除没有双频载波相位的观测值和双频伪距相差过大的观测值，原路返回的 `obs` 筛选过后的观测值列表。

```c
// 筛选出能进行 PPP 解算的观测值
extern void obsScan_PPP(const prcopt_t *popt, obsd_t *obs, const int nobs, int *nValid)
{
	int i,n,sat,f2;

	// 遍历传入的观测值列表，剔除没有双频载波相位的观测值和双频伪距相差过大的观测值
	for (i=n=0;i<nobs&&i<MAXOBS;i++) {
		sat=obs[i].sat;

		f2=1;
		//if (NFREQ>=3&&(PPP_Glo.sFlag[sat-1].sys&(SYS_GAL|SYS_SBS))) f2=2;
		if (popt->mode>=PMODE_PPP_KINEMA) {

			// 剔除没有双频载波相位的观测值
			if (obs[i].L[0]*obs[i].L[f2]==0.0) continue;
		}

		// 剔除双频伪距相差过大的观测值
		if (fabs(obs[i].P[0]-obs[i].P[f2])>=200.0) continue;

		// 返回的 obs 筛选过后的观测值列表
		obs[n]=obs[i];
		n++;
	}

	if (nValid) *nValid=n;
}
```

#### 3. calDop()：调用 dops() 计算各种 DOP 值

遍历传入的观测值列表 `obs`，记录各系统的有效卫星数 `rtk->sol.ns`、高度角方位角 `rtk->ssat[sat-1].azel`，调用 `dops()` 计算各种 DOP 值，存下并返回 PDOP。

```c
static double calDop(rtk_t *rtk, const obsd_t *obs, const int n)
{
	double azel[MAXSAT*2],dop[4];
	int i,num,sat;

    for (i=0;i<NSYS_USED;i++) rtk->sol.ns[i]=0;

	// 遍历传入的观测值列表 obs，记录各系统的有效卫星数 rtk->sol.ns、高度角方位角 rtk->ssat[sat-1].azel
	for (i=num=0;i<n;i++) {
		sat=obs[i].sat;
		if (rtk->ssat[sat-1].vsat[0]==0) continue;
        if (PPP_Glo.sFlag[sat-1].sys==SYS_GPS) rtk->sol.ns[0]++;
        else if (PPP_Glo.sFlag[sat-1].sys==SYS_GLO) rtk->sol.ns[1]++;
        else if (PPP_Glo.sFlag[sat-1].sys==SYS_CMP) rtk->sol.ns[2]++;
        else if (PPP_Glo.sFlag[sat-1].sys==SYS_GAL) rtk->sol.ns[3]++;
        else if (PPP_Glo.sFlag[sat-1].sys==SYS_QZS) rtk->sol.ns[4]++;
		azel[2*num+0]=rtk->ssat[sat-1].azel[0];
		azel[2*num+1]=rtk->ssat[sat-1].azel[1];
		num++;
	}

	// 调用 dops 计算各种 DOP 值
	dops(num,azel,0.0,dop);

	rtk->sol.dop[1]=dop[1];	// 存下 PDOP

	return dop[1];	// 返回 PDOP
}
```

#### 4. dops()：计算各种 DOP 值

根据卫星高度角、方位角构建 H 矩阵，$H * H^T$ 得到 Q 矩阵，之后计算：

* 几何精度因子： $G D O P=\sqrt{q_{11}+q_{22}+q_{33}+q_{44}}$
* 空间位置精度因子： $P D O P=\sqrt{q_{11}+q_{22}+q_{33}}$
* 平面位置精度因子：$H D O P=\sqrt{q_{11}^{\prime}+q_{22}^{\prime}}$

* 高程精度因子：$ V D O P=\sqrt{q_{33}^{\prime}}$

> 没算接收机钟差精度因子：$T D O P=\sqrt{q_{44}}$

```c
extern void dops(int ns, const double *azel, double elmin, double *dop)
{
    double H[4*MAXSAT],Q[16],cosel,sinel;
    int i,n;
    
    for (i=0;i<4;i++) dop[i]=0.0;

    // 根据卫星高度角、方位角构建 H 矩阵
    for (i=n=0;i<ns&&i<MAXSAT;i++) {
        if (azel[1+i*2]<elmin||azel[1+i*2]<=0.0) continue;
        cosel=cos(azel[1+i*2]);
        sinel=sin(azel[1+i*2]);
        H[  4*n]=cosel*sin(azel[i*2]);
        H[1+4*n]=cosel*cos(azel[i*2]);
        H[2+4*n]=sinel;
        H[3+4*n++]=1.0;
    }
    if (n<4) return;
    
    // H * H^T 得到 Q 矩阵
    matmul("NT",4,4,n,1.0,H,H,0.0,Q);
    if (!matinv(Q,4)) {
        dop[0]=SQRT(Q[0]+Q[5]+Q[10]+Q[15]); /* GDOP */
        dop[1]=SQRT(Q[0]+Q[5]+Q[10]);       /* PDOP */
        dop[2]=SQRT(Q[0]+Q[5]);             /* HDOP */
        dop[3]=SQRT(Q[10]);                 /* VDOP */
    }
}
```

#### 5. keepEpInfo()：保存当前历元信息

```cpp
extern void keepEpInfo(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
	int i,j,sat;
	prcopt_t *opt=&rtk->opt;
	double wl0,wl1,var0,var1,gf;

	for (i=0;i<MAXSAT;i++) {
		rtk->ssat[i].gf=0.0;
		PPP_Glo.ssat_Ex[i].mw[0]=0.0;
	}

	// 遍历传入的 obs 观测值列表
	for (i=0;i<n&&i<MAXOBS;i++) {
		sat=obs[i].sat;

		PPP_Glo.ssat_Ex[sat-1].tLast=PPP_Glo.tNow;

		//if ( rtk->ssat[sat-1].azel[1]<rtk->opt.elmin ) 
		//	continue;

		// 
		if ((gf=gfmeas(obs+i,nav))!=0.0)
			rtk->ssat[sat-1].gf=gf;

		// 计算 WL 组合，为了后面计算 MW 组合
		if ((wl1=wlAmbMeas(obs+i,nav))==0.0)
			continue;

		wl0=PPP_Glo.ssat_Ex[sat-1].mw[1];
		PPP_Glo.ssat_Ex[sat-1].mw[0]=wl1;

		if (PPP_Glo.ssat_Ex[sat-1].mwIndex>0) {
			j=PPP_Glo.ssat_Ex[sat-1].mwIndex;
			var0=PPP_Glo.ssat_Ex[sat-1].mwVar_c;
			var1=(wl1-wl0)*(wl1-wl0)-var0;
			var1=var0 + var1/j;

			PPP_Glo.ssat_Ex[sat-1].mw[1]=(wl0*j+wl1)/(j+1);
			PPP_Glo.ssat_Ex[sat-1].mwIndex++;
			PPP_Glo.ssat_Ex[sat-1].mwVar_c=var1;
		}
		else {
			PPP_Glo.ssat_Ex[sat-1].mw[1]=wl1;
			PPP_Glo.ssat_Ex[sat-1].mwIndex++;
			PPP_Glo.ssat_Ex[sat-1].mwVar_c=0.25;
		}

		j=IB(sat,0,opt);

		PPP_Glo.ssat_Ex[sat-1].arc.ifArc_m=rtk->x[j];
		PPP_Glo.ssat_Ex[sat-1].arc.ifVarArc_m=rtk->P[j*rtk->nx+j];

		PPP_Glo.ssat_Ex[sat-1].arc.mwArc_c=PPP_Glo.ssat_Ex[sat-1].mw[1];
		PPP_Glo.ssat_Ex[sat-1].arc.mwArcVar_c=PPP_Glo.ssat_Ex[sat-1].mwVar_c;
	}
}
```

#### 6. gfmeas()：计算 GF 几何无关组合观测值

$$
L_{\mathrm{GF}}(i)=\lambda_{1} \Phi_{1}(i)-\lambda_{2} \Phi_{2}(i)
$$

```c
extern double gfmeas(const obsd_t *obs, const nav_t *nav)
{
	const double *lam=nav->lam[obs->sat-1];

	if (lam[0]==0.0||lam[1]==0.0||obs->L[0]==0.0||obs->L[1]==0.0) return 0.0;

	return lam[0]*obs->L[0]-lam[1]*obs->L[1];
}
```

#### 7. wlAmbMeas()：计算 WL 宽巷组合观测值

$$
\varphi_{1}-\varphi_{2}-\frac{f_{1}-f_{2}}{f_{1}+f_{2}}\left(\frac{P_{1}}{\lambda_{1}}+\frac{P_{2}}{\lambda_{2}}\right)
$$

```c
extern double wlAmbMeas(const obsd_t *obs, const nav_t *nav)
{
	int i=0,j=1;
	const double *lam=nav->lam[obs->sat-1];
	double P1,P2,P1_C1,P2_C2,lam1,lam2,res;

	if (obs->L[i]==0.0) return 0.0;
	if (obs->L[j]==0.0) return 0.0;
	if (obs->P[i]==0.0) return 0.0;
	if (obs->P[j]==0.0) return 0.0;
	if (lam[i]*lam[j]==0.0) return 0.0;

	P1=obs->P[i];
	P2=obs->P[j];
	P1_C1=nav->cbias[obs->sat-1][1];
	P2_C2=nav->cbias[obs->sat-1][2];

	if (obs->code[0]==CODE_L1C) P1+=P1_C1; /* C1->P1 */
	if (obs->code[1]==CODE_L2C) P2+=P2_C2; /* C2->P2 */

	lam1=lam[i];
	lam2=lam[j];
	res=(obs->L[i]-obs->L[j])-(lam2-lam1)/(lam1+lam2)*(P1/lam1+P2/lam2);

	return res;
}
```
