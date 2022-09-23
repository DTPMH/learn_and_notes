<center><span style="font-size:2rem;font-weight:bold;">C++输出n位有效数字</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 包含的头文件

`#include<iomanip>`

# 用法

|     控制符      |              描述              |
| :-------------: | :----------------------------: |
| setprecision(n) |   将一个浮点数的精度设置为n    |
|      fixed      | 将一个浮点数以定点数的格式输出 |

## `setprecision(n)`

**1.指定一个浮点数打印几位数字，其中n是总位数(包括小数之前和之后的数字),超出的位数会被四舍五入进去。**
**2.setprecision(n)一直作用到下一个setprecisin(n)之前。**

```C++
#include<iostream>
#include<iomanip>
using namespace std;
int main()
{
	double pi = 3.1415926;
	cout<<setprecision(4)<<pi<<endl;  //实现保留4位有效数字；    输出：3.142
	cout<<pi<<endl;                   //setprecision(n)持续作用；输出：3.142
	return 0;
}
```

## fixed

**fixed：用一般的方式输出浮点数，而不是科学计数法。**
**思路：fixed与setprecision(n)连用**

```C++
#include<iostream>
#include<iomanip>
using namespace std;
int main()
{
	double pi = 3.1415926;
	cout<<fixed<<setprecision(4)<<pi<<endl; 
	//实现保留四位小数；输出：3.1416
	//fixed与setprecision(n)位置可以互换
	return 0;
}
```

