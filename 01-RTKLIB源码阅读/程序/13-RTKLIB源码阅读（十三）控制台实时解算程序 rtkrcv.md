[TOC]

## 一、rtkrcv 用法

### 1、简介





### 2、配置文件





### 3、命令行参数

*     `-s`：start RTK server on program startup
*     `-p port`：port number for telnet console
*     `-m port`：port number for monitor stream
*     `-d dev`：terminal device for console
*     `-o file`：processing options file
*     `-w pwd`：login password for remote console ("": no password)
*     `-r level`：output solution status file (0:off,1:states,2:residuals)
*     `-t level`：debug trace level (0:off,1-5:on)
*     `-sta sta`：station name for receiver dcb



### 4、终端命令

* `start`：
* `stop`：
* `restart`：
* `solution [cycle]`：
* `status [cycle]`：
* `satellite [-n] [cycle]`：
* `observ [-n] [cycle]`：
* `navidata [cycle]`：
* `stream [cycle]`：
* `error`：
* `option [opt]`：
* `set opt [val]`：
* `load [file]`：
* `save [file]`：
* `log [file|off]`：
* `help|? [path]`：
* `exit`：
* `shutdown`：
* `!command [arg...]`：











## 三、rtksvr.c 的函数









## 四、rtkrcv.c/vt.c/vt.h 的类型、函数

### 1、虚拟终端

#### 1. 虚拟终端结构体 vt_t



```c
typedef struct vt_tag {                 /* virtual console type */          // 
    int state;                          /* state(0:close,1:open) */         // (0:close,1:open)
    int type;                           /* type (0:dev,1:telnet) */         // (0:dev,1:telnet)
    int in,out;                         /* input/output file descriptor */  // 输入、输出文件描述符
    int n,nesc;                         /* number of line buffer/escape */  // 当前行数
    int cur;                            /* cursor position */               // 光标位置
    int cur_h;                          /* current history */               // 光标历史位置
    int brk;                            /* break status */                  // 
    int blind;                          /* blind inpu mode */               // 屏蔽输入模式
    struct termios tio;                 /* original terminal attribute */   // 
    char buff[MAXBUFF];                 /* line buffer */                   // 
    char esc[8];                        /* escape buffer */                 // 
    char *hist[MAXHIST];                /* history buffer */                // 历史字符串
    FILE *logfp;                        /* log file pointer */              // log 文件指针
} vt_t;
```

#### 2. 操作虚拟终端的基础函数



```c
extern vt_t *vt_open(int sock, const char *dev);
extern void vt_close(vt_t *vt);
extern int vt_getc(vt_t *vt, char *c);
extern int vt_gets(vt_t *vt, char *buff, int n);
extern int vt_putc(vt_t *vt, char c);
extern int vt_puts(vt_t *vt, const char *buff);
extern int vt_printf(vt_t *vt, const char *format, ...);
extern int vt_chkbrk(vt_t *vt);
extern int vt_openlog(vt_t *vt, const char *file);
extern void vt_closelog(vt_t *vt);
```

#### 6. 虚拟终端输出函数



```c
static void prtime(vt_t *vt, gtime_t time);	//输出时间
static void prsolution(vt_t *vt, const sol_t *sol, const double *rb);	//输出结果
static void prstatus(vt_t *vt);				// 输出状态
static void prsatellite(vt_t *vt, int nf);	// 输出卫星数据
static void probserv(vt_t *vt, int nf);		// 输出观测值数据
static void prnavidata(vt_t *vt);			// 输出星历数据
static void prerror(vt_t *vt);				// 输出错误信息
static void prstream(vt_t *vt);				// 输出数据流
static void prssr(vt_t *vt);				// 输出 SSR 改正数据
```

#### 5. 虚拟终端控制结构体 con_t



```c
typedef struct {                       /* console type */
    int state;                         /* 状态 (0:stop,1:run) */
    vt_t *vt;                          /* 虚拟终端结构体 */
    pthread_t thread;                  /* 线程控制 */
} con_t;
```

#### 6. 处理终端命令函数



```c
cmd_start    (char **args, int narg, vt_t *vt);
cmd_stop     (char **args, int narg, vt_t *vt);
cmd_restart  (char **args, int narg, vt_t *vt);
cmd_solution (char **args, int narg, vt_t *vt);
cmd_status   (char **args, int narg, vt_t *vt);
cmd_satellite(char **args, int narg, vt_t *vt);
cmd_observ   (char **args, int narg, vt_t *vt);
cmd_navidata (char **args, int narg, vt_t *vt);
cmd_stream   (char **args, int narg, vt_t *vt);
cmd_ssr      (char **args, int narg, vt_t *vt);
cmd_error    (char **args, int narg, vt_t *vt); 
cmd_option   (char **args, int narg, vt_t *vt);
cmd_set      (char **args, int narg, vt_t *vt); 
cmd_load     (char **args, int narg, vt_t *vt); 
cmd_save     (char **args, int narg, vt_t *vt); 
cmd_log      (char **args, int narg, vt_t *vt); 
cmd_help     (char **args, int narg, vt_t *vt); 
cmd_help     (char **args, int narg, vt_t *vt); 
```





















## 六、程序执行流程

### 1、线程设计



### 2、程序执行流程图





### 3、主要函数



























































