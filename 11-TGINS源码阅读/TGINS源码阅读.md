



代码风格比较 C 语言，用了一些 C++ 



直接运行终端会输出很多很多中间数据，有些调试输出没删干净，先全局搜索 matprint 把解算代码中的都注释掉。



命令行参数：



重点文件：







重点关注函数：

* main()：
* loadMeasurement()：
* multiSensorFusing()：
* sensorFusingProcess()：
* initProcess()：
* insAlign()：INS 递推
  * insUpdate()：
    * calibrateImu()：
    * insUpdateECEF()：
    * insUpdateLLH()：
    * updateEarthPar()：
  * transferMatrix()：updateF_ECEF() 实现 e 系计算、updateF_LLH() 实现 n 系计算
  * predicted()：调用 updateP() 更新协方差
* inputImu()：
* processTimeControl()：
* timeUpdate()：
* syncSensors()：
* 





记录一次调试经历：

* 写输出代码之前先试试在前面打个断点，确认程序能运行到那

* 在 insFeedback() 开头可以输出误差反馈的增量：

  ![1697883132(1)](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1697883132(1).png)

* 





matprint()：

* trans：-1 退出不输出、0：不转置输出、1：转置输出
* A：矩阵
* n：行
* m：列
* p：输出数字总长度
* q：输出数字小数位数
* remark：输出矩阵前先输出的一行字符串，传 null 不输出

```c
/* print matrix ----------------------------------------------------------------
* print matrix to stdout
* args   : double *A        I   matrix A (n x m)
*          int    n,m       I   number of rows and columns of A
*          int    p,q       I   total columns, columns under decimal point
*         (FILE  *fp        I   output file pointer)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void matfprint(const double A[], int n, int m, int p, int q, FILE *fp)
{
    int i,j;
    
    for (i=0;i<n;i++) {
        for (j=0;j<m;j++) fprintf(fp," %*.*G",p,q,A[i+j*n]);
        fprintf(fp,"\n");
    }
}
extern void matprint(int trans,const double A[], int n, int m, int p, int q,const char *remark)
{
    int i,j;
    double *B=NULL;
    if(trans<0) return;
    if(remark) fprintf(stdout,"%s\n",remark);
    if(trans){
        B=mat(m,n);
        for(i=0;i<n;i++){
            for(j=0;j<m;j++){
                B[j+i*m]=A[i+j*n];
            }
        }
        matfprint(B,m,n,p,q,stdout);
        free(B);
    }
    else{
        matfprint(A,n,m,p,q,stdout);
    }
    fflush(stdout);
}
```









