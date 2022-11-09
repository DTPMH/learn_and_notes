<center><span style="font-size:2rem;font-weight:bold;">C++随机数生成</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# C语言与C++可通用的随机数生成

## 伪随机数生成

```C++
#include<stdlib>
rand()%100;//生成0-99的随机数，但是这种方法每次生成的随机数都是一样的，因此属于伪随机数
```

## 随机数生成

```C++
#include<stdlib>
#include<time.h>

srand((int)time(0));
rand()%100;//生成0-99的随机数
```

# C++11引入随机数生成库

## 默认随机数，产生随机非负数

```C++
#include<random>
#include<ctime>

std::default_random_engine e;
e.seed(time(0));
e();////生成随机非负数
```

## 产生均匀分布的整数

```C++
#include<random>
#include<ctime>

std::default_random_engine e;
std::uniform_int_distribution<int> u(0,20);//[0-20]闭区间
e.seed(time(0));[
u(e);//生成[0-20]均匀分布的随机整数
```

## 产生均匀分布的随机实数

```C++
#include<random>
#include<ctime>

std::default_random_engine e;
std::uniform_real_distribution<double> u(1.5,19.5);//[1.5,19.5]闭区间
e.seed(time(0));
u(e);//生成[1.5,19.5]闭区间均匀分布的随机实数
```

## 产生正态分布的实数

```C++
#include<random>
#include<ctime>

std::default_random_engine e;
std::normal_distribution<double> u(0,1);//产生均值为0，标准差为的随机实数
e.seed(time(0));
u(e);//产生均值为0，标准差为的随机实数
```

## 生成二项分布的bool值

```C++
#include<random>
#include<ctime>

std::default_random_engine e;
std::bernoulli_distribution u(0.8);//生成1的概率为0.8
e.seed(time(0));
u(e);//随机生成0,1，生成1的概率为0.8
```

