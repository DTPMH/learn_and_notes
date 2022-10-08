<center><span style="font-size:2rem;font-weight:bold;">C++ Vector的使用</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

#  头文件

使用vector需要包含头文件，且需要用到std

`std::vector<int>`

或者

`using namespace std;`

# vector简介

vector是一个动态的序列容器，相当于一个size可变的数组。

# 常见用法

## 声明以及初始化

### vector的声明

```c++
vector<int> a;           //声明一个int型向量a,没有指定大小
vector<int> a(10);       //声明一个初始大小为10的int型向量
vector<int> a(10,1);     //声明一个初始大小为10且初始值为1的向量
vector<int> b(a);        //声明一个int型向量b，并且用向量a初始化向量b
vector<int> b(a.begin().a.begin()+3);       //声明一个int型向量b,将a[0]-a[2]作为b向量的初值
vector<int> 
```

### vector的初始化

```C++
int n[]={1,2,3,4,5};
vector<int> a(n,n+5);        //将数组n的前5个元素作为向量a的初值
vector<int> a(&n[1],&n[4]);  //将n[1]-n[4]范围内的元数作为向量a的初始值
vector<char> strings={'a','b','c','d'};
vector<int> value={1,2,3,4,5};
```

## 插入元素

### push_back函数

```C++
vector<int> a;
a.push_back(1); 
```

### insert函数

#### 例子

```C++
a.insert(a.begin(),0);                      //在a[0]前插入0
a.insert(a.begin()+1,5);                    //在a[1]处插入5
a.insert(a.begin(),3,5);                    //在a[0]处插入3个数，每个数均为5
a.insert(a.begin(),b.begin(),b.end())；       //在a[0]处插入b向量
a.insert(a.begin(),b.begin()+1,b.begin()+5)   //在a[0]处插入b[1]-b[4]
```

## 删除元素

```C++
a.pop_back();                       //删除最后一个元素
a.erase(a.begin());                 //删除第一个元素
a.erase(a.begin()+1,a.end());       //删除a[1]-最后一个元素
a.clear();                          //清空a向量
```

## 遍历函数

```C++
a.front();                        //指向a的第一个元素
a.back();						  //指向a的最后一个元素
a.begin();						  //指向a向量的头指针，指向第一个元素，类型是迭代器(iterator)
a.end();						  //返回a向量的尾指针，指向最后一个元素的下一个位置
a.rbegin();                       //反向迭代器，指向最后一个元素的指针
a.rend();                         //反向迭代器，指向第一个元素之前的指针
a.at(pos);						  //返回处于pos的元素，pos是int型
```

## 向量元素的赋值

```C++
vector<int> a={1,2,3,34,4};
a.assign(b.begin(),b.begin()+3);                //将b[0]-b[2]赋值给a
a.assign(4,2);                                  //使a只含4各元素，且每个元素为2
a.swap(b);                                      //将a向量与b向量中的元素整体互换
a=b;                                            //将b向量中的元素赋给a向量
```

## 判断向量是否为空

```C++
a.empty();
```

## 判断向量的大小

```C++
a.size();                                //返回向量中的元素的个数
a.capacity();							//返回向量中所能容纳的最大元素值
a.maxsize();							//返回最大可允许的向量元素数量值
```

## 对向量中的元素进行排序

用到排序需要包含头文件：`<algorithm>`

```C++
sort(a.begin(),a.end());                       //将a向量降序排列
```

更多关于sort的知识，请见[跳转](E:\梯梯\工作收获\xgd工作收获\learn_and_notes\学习提升\code学习\C++学习\c++中排序sort函数总结.md)

## 元素倒置

必须包含头文件：`<algorithm>`

```C++
reverse(a,begin().a.end());                 //将a向量中的元素倒置
```

## 查找元素

必须包含头文件：`<algorithm>`

```C++
a.find(a.begin().a.end(),3);              //从a向量中寻找元素3,若存在返回其在向量中的位置，若不存在返回a.end()
```

## 调整向量的容量

```C++
a.resize(10);           //将a向量的元素个数调整为10，多的删除，不够的补足0
a.resize(10,2);         //将a向量的元素个数调整为10，多的删除，不够的补2
a.resize(1000);         //将a向量的元素个数调整为1000
```

## 创建二维向量

```C++
vector<vector<int> >Matrix_2(10,vector<int>(5,1));                      //返回10x5的二维数组，每个元素均为1
```

