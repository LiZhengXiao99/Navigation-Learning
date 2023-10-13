> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、矩阵计算

### 1、矩阵计算相关函数

GNSS 处理的数据都是矩阵数据，RTKLIB 的 **rtkcmn.c** 中写了一些矩阵运算的函数。

1. **mat()** ：New matrix、创建新矩阵，传入行数n，列数m，malloc开辟n*m个**double**空间，返回此空间的首地址。

   ```C++
   extern double *mat(int n, int m)
   {
       double *p;
       
       if (n<=0||m<=0) return NULL;
       if (!(p=(double *)malloc(sizeof(double)*n*m))) {
           fatalerr("matrix memory allocation error: n=%d,m=%d\n",n,m);
       }
       return p;
   }
   ```

2. **imat()、**New integer matrix、与mat类似，区别在于iamt创建的矩阵是**int**类型的。**（RTKLIB里的矩阵一般都是double类型的二维数组）**

   ```C++
   extern int *imat(int n, int m)
   {
       int *p;
       
       if (n<=0||m<=0) return NULL;
       if (!(p=(int *)malloc(sizeof(int)*n*m))) {
           fatalerr("integer matrix memory allocation error: n=%d,m=%d\n",n,m);
       }
       return p;
   }
   ```

3. **zeros():**New zero matrix、创建0矩阵、**double**类型。它比mat多了`\#if NOCALLOC if ((p=mat(n,m))) for (n=n*m-1;n>=0;n--) p[n]=0.0;`，如果预编译指令有NOCALLOC，会把所有位赋值0.0

   ```C++
   extern double *zeros(int n, int m)
   {
       double *p;
       
   #if NOCALLOC
       if ((p=mat(n,m))) for (n=n*m-1;n>=0;n--) p[n]=0.0;
   #else
       if (n<=0||m<=0) return NULL;
       if (!(p=(double *)calloc(sizeof(double),n*m))) {
           fatalerr("matrix memory allocation error: n=%d,m=%d\n",n,m);
       }
   #endif
       return p;
   }
   ```

4. **eye()**：New identity matrix、创建单位矩阵，**对角线元素全赋值1.0** 。

   ```C++
   extern double *eye(int n)
   {
       double *p;
       int i;
       
       if ((p=zeros(n,n))) for (i=0;i<n;i++) p[i+i*n]=1.0;
       return p;
   }
   ```

5. **dot()**：Inner Product、点乘、求两个向量的**内积**（对应位元素相乘再相加）

   ```C++
   extern double dot(const double *a, const double *b, int n)
   {
       double c=0.0;
       
       while (--n>=0) c+=a[n]*b[n];
       return c;
   }
   ```

6. **norm()**：Euclid norm、求向量的I2范数（向量自己的内积再开方 ）

   ```c++
   extern double norm(const double *a, int n)
   {
       return sqrt(dot(a,a,n));
   }
   ```

   . **cross3()**	、Outer product of 3D vectors、三维向量的外积

   ```c++
   extern void cross3(const double *a, const double *b, double *c)
   {
       c[0]=a[1]*b[2]-a[2]*b[1];
       c[1]=a[2]*b[0]-a[0]*b[2];
       c[2]=a[0]*b[1]-a[1]*b[0];
   }
   ```

   . **normv3()**	Normalize 3D vector、规格化三维向量（各元素除以向量的I2范数），a为传入向量，b为传出向量。

   ```c++
   extern int normv3(const double *a, double *b)
   {
       double r;
       if ((r=norm(a,3))<=0.0) return 0;
       b[0]=a[0]/r;
       b[1]=a[1]/r;
       b[2]=a[2]/r;
       return 1;
   }
   ```

7. **matcpy()：**Copy matrix、将B矩阵的所有元素赋值给A矩阵 

   ```C++
   extern void matcpy(double *A, const double *B, int n, int m)
   {
       memcpy(A,B,sizeof(double)*n*m);
   }
   ```

8. **matmul()：**Multiply matrix、矩阵A、B相乘返回C

   - tr：两个元素的字符数组，Tr、Tr[1]分别标识A、B的状态，取值N正常、T转置，可直接写TN、NN。
   - n、k、m：A为nm矩阵， B为mk矩阵、A的列数与B的行数相等，所有只用三个参数。作者之所以把n和k放在m前面，看起来顺序反逻辑，其实是矩阵相乘一共要循环n*k次，从编程的顺序来排的 。
   - 总体流程为： C=alphaAB+beta*C 

   ```c++
   extern void matmul(const char *tr, int n, int k, int m, double alpha,
                      const double *A, const double *B, double beta, double *C)
   {
       double d;
       int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);
       
       for (i=0;i<n;i++) for (j=0;j<k;j++) {
           d=0.0;
           switch (f) {
               case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
               case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
               case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
               case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
           }
           if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
       }
   }
   ```

9. **matinv()：**Inverse of matrix、矩阵求逆

   ```C++
   extern int matinv(double *A, int n)
   {
       double d,*B;
       int i,j,*indx;
       
       indx=imat(n,1); B=mat(n,n); matcpy(B,A,n,n);
       if (ludcmp(B,n,indx,&d)) {free(indx); free(B); return -1;}
       for (j=0;j<n;j++) {
           for (i=0;i<n;i++) A[i+j*n]=0.0;
           A[j+j*n]=1.0;
           lubksb(B,n,indx,A+j*n);
       }
       free(indx); free(B);
       return 0;
   }
   ```

10. **solve()：**Solve linear equation、求方程线性解 、AX=Y，求X，A为nn，Y为nm

    ```c++
    extern int solve(const char *tr, const double *A, const double *Y, int n,
                     int m, double *X)
    {
        double *B=mat(n,n);
        int info;
        
        matcpy(B,A,n,n);
        if (!(info=matinv(B,n))) matmul(tr[0]=='N'?"NN":"TN",n,m,n,1.0,B,Y,0.0,X);
        free(B);
        return info;
    }
    ```

    

11. **ludcmp ()**：LU分解，即把矩阵A分解LU乘积的形式，U为单位上三角矩阵和L单位为下三角矩阵两部分 。

    ```c++
    static int ludcmp(double *A, int n, int *indx, double *d)
    {
        double big,s,tmp,*vv=mat(n,1);
        int i,imax=0,j,k;
        
        *d=1.0;
        for (i=0;i<n;i++) {
            big=0.0; for (j=0;j<n;j++) if ((tmp=fabs(A[i+j*n]))>big) big=tmp;
            if (big>0.0) vv[i]=1.0/big; else {free(vv); return -1;}
        }
        for (j=0;j<n;j++) {
            for (i=0;i<j;i++) {
                s=A[i+j*n]; for (k=0;k<i;k++) s-=A[i+k*n]*A[k+j*n]; A[i+j*n]=s;
            }
            big=0.0;
            for (i=j;i<n;i++) {
                s=A[i+j*n]; for (k=0;k<j;k++) s-=A[i+k*n]*A[k+j*n]; A[i+j*n]=s;
                if ((tmp=vv[i]*fabs(s))>=big) {big=tmp; imax=i;}
            }
            if (j!=imax) {
                for (k=0;k<n;k++) {
                    tmp=A[imax+k*n]; A[imax+k*n]=A[j+k*n]; A[j+k*n]=tmp;
                }
                *d=-(*d); vv[imax]=vv[j];
            }
            indx[j]=imax;
            if (A[j+j*n]==0.0) {free(vv); return -1;}
            if (j!=n-1) {
                tmp=1.0/A[j+j*n]; for (i=j+1;i<n;i++) A[i+j*n]*=tmp;
            }
        }
        free(vv);
        return 0;
    }
    ```

    

12. **lubksb ()**：LU回代,即把单位上三角矩阵U和单位下三角矩阵L矩阵回代为一个整体矩阵 

    ```c++
    static void lubksb(const double *A, int n, const int *indx, double *b)
    {
        double s;
        int i,ii=-1,ip,j;
        
        for (i=0;i<n;i++) {
            ip=indx[i]; s=b[ip]; b[ip]=b[i];
            if (ii>=0) for (j=ii;j<i;j++) s-=A[i+j*n]*b[j]; else if (s) ii=i;
            b[i]=s;
        }
        for (i=n-1;i>=0;i--) {
            s=b[i]; for (j=i+1;j<n;j++) s-=A[i+j*n]*b[j]; b[i]=s/A[i+i*n];
        }
    }
    ```

    

13. **matprint()：**Print matrix、打印矩阵到stdout

    ```c++
    extern void matprint(const double A[], int n, int m, int p, int q)
    {
        matfprint(A,n,m,p,q,stdout);
    }
    ```

    

14. **matfprint()：**Print matrix to file、打印矩阵到文件中

```c++
extern void matfprint(const double A[], int n, int m, int p, int q, FILE *fp)
{
    int i,j;
    
    for (i=0;i<n;i++) {
        for (j=0;j<m;j++) fprintf(fp," %*.*f",p,q,A[i+j*n]);
        fprintf(fp,"\n");
    }
}
```



### 2、最小二乘与Kalman滤波

1. **lsq()：**Least square estimation、最小二乘估计

   - **A**：nm阶设计矩阵的转置，m<n则无法计算。
   - **y**：m阶观测残差，**y=v=l-HX** 。
   - **X**：传出参数、待估计的n阶参数向量的增量。
   - **Q**：传出参数、nn协方差阵。

   ```c++
   extern int lsq(const double *A, const double *y, int n, int m, double *x,
                  double *Q)
   {
       double *Ay;
       int info;
       
       if (m<n) return -1;
       Ay=mat(n,1);
       matmul("NN",n,1,m,1.0,A,y,0.0,Ay); /* Ay=A*y */
       matmul("NT",n,n,m,1.0,A,A,0.0,Q);  /* Q=A*A' */
       if (!(info=matinv(Q,n))) matmul("NN",n,1,n,1.0,Q,Ay,0.0,x); /* x=Q^-1*Ay */
       free(Ay);
       return info;
   }
   ```

2. **filter_()、filter():**卡尔曼滤波器

   - X：n阶参数向量，就是参数，不再是最小二乘中的参数的增量 
   - P：nn阶协方差阵。X、P在filter()函数中既是传入参数，也是传出参数，迭代。
   - H：nm阶设计矩阵的转置
   - V：m阶新息向量
   - R：mm阶量测噪声协方差阵
   - Xp、Pp：预测参数和预测协方差

   ```c++
   extern  int filter_(const double *x, const double *P, const double *H,
                      const double *v, const double *R, int n, int m,
                      double *xp, double *Pp)
   {
       double *F=mat(n,m),*Q=mat(m,m),*K=mat(n,m),*I=eye(n);
       int info;
       
       matcpy(Q,R,m,m);
       matcpy(xp,x,n,1);
       matmul("NN",n,m,n,1.0,P,H,0.0,F);       /* Q=H'*P*H+R */
       matmul("TN",m,m,n,1.0,H,F,1.0,Q);
       if (!(info=matinv(Q,m))) {
           matmul("NN",n,m,m,1.0,F,Q,0.0,K);   /* K=P*H*Q^-1 */
           matmul("NN",n,1,m,1.0,K,v,1.0,xp);  /* xp=x+K*v */
           matmul("NT",n,n,m,-1.0,K,H,1.0,I);  /* Pp=(I-K*H')*P */
           matmul("NN",n,n,n,1.0,I,P,0.0,Pp);
       }
       free(F); free(Q); free(K); free(I);
       return info;
   }
   ```

   ```c++
   extern int filter(double *x, double *P, const double *H, const double *v,
                     const double *R, int n, int m)
   {
       double *x_,*xp_,*P_,*Pp_,*H_;
       int i,j,k,info,*ix;
       
       ix=imat(n,1); for (i=k=0;i<n;i++) if (x[i]!=0.0&&P[i+i*n]>0.0) ix[k++]=i;
       x_=mat(k,1); xp_=mat(k,1); P_=mat(k,k); Pp_=mat(k,k); H_=mat(k,m);
       for (i=0;i<k;i++) {
           x_[i]=x[ix[i]];
           for (j=0;j<k;j++) P_[i+j*k]=P[ix[i]+ix[j]*n];
           for (j=0;j<m;j++) H_[i+j*k]=H[ix[i]+j*n];
       }
       info=filter_(x_,P_,H_,v,R,k,m,xp_,Pp_);
       for (i=0;i<k;i++) {
           x[ix[i]]=xp_[i];
           for (j=0;j<k;j++) P[ix[i]+ix[j]*n]=Pp_[i+j*k];
       }
       free(ix); free(x_); free(xp_); free(P_); free(Pp_); free(H_);
       return info;
   }
   ```

3. **smoother()：**Kalman smoother 、结合前向滤波和后向滤波的卡尔曼滤波平滑

   - xf、Qf：前向滤波n阶结果和nn阶协方差
   - xb、Qb：后向滤波n阶结果和nn阶协方差
   - xs、Qs：平滑n阶结果和nn阶协方差

   ```c++
   extern int smoother(const double *xf, const double *Qf, const double *xb,
                       const double *Qb, int n, double *xs, double *Qs)
   {
       double *invQf=mat(n,n),*invQb=mat(n,n),*xx=mat(n,1);
       int i,info=-1;
       
       matcpy(invQf,Qf,n,n);
       matcpy(invQb,Qb,n,n);
       if (!matinv(invQf,n)&&!matinv(invQb,n)) {
           for (i=0;i<n*n;i++) Qs[i]=invQf[i]+invQb[i];
           if (!(info=matinv(Qs,n))) {
               matmul("NN",n,1,n,1.0,invQf,xf,0.0,xx);
               matmul("NN",n,1,n,1.0,invQb,xb,1.0,xx);
               matmul("NN",n,1,n,1.0,Qs,xx,0.0,xs);
           }
       }
       free(invQf); free(invQb); free(xx);
       return info;
   }
   ```

