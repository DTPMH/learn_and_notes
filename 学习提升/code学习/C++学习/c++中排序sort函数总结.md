<center><span style="font-size:2rem;font-weight:bold;">C++中的sort函数总结</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 简介

C++中的sort()函数是C++标准库STL中的排序算法，是一种对数组与向量vector排序的比较快的算法（也可以用于list容器），且写法比较简单，容易理解。（但是排序方式与priority_queue是相反的，具体看另一篇关于优先级队列的排序教程）

# 包含的头文件

`#include<algorithm>`

# 默认的排序方式

默认的排序方式是升序排列，即从小到大排序

例子

```C++
#include<iostream>
#include<algorithm>
#include<vector>
Int main()
{
    vector<int> a={5,20,25,45,68,21,55,36,100};
    sort(a.begin(),a.end();//升序，从小到大
    for(int i=0;i<a.size();i++)
    {
        cout<<a[i]<<endl;
    }
}

```

# 自定义的排序方式

## 库中自带的排序方式

Sort()函数实际上有三个参数：sort(start,end,排序方法)；

**注意： 库中有的greater<int>() //（降序，从大到小）；less<int>() //(升序，从小到大)**

```C++
int main()
{
    
    vector<int> a={5,20,25,45,68,21,55,36,100};
    /*for(int i=0;i<10;i++)
    {
        a.push_back(i);
    }
    
    sort(a.begin(),a.end(),greater<int>());//降序,从大到小
    //sort(a.begin(),a.end(),less<int>());//升序，从小到大
    for(int i=0;i<a.size();i++)
    {
        cout<<a[i]<<endl;
}
```

**注意：其中greater<int>()，以及less<int>()，中的int表示数组或者向量中的数据类型，也可以为：double,string,flost,pair<int,int>….等**

## 自定义比较函数

**注意：自己定义的比较函数类型必须是bool型的**

例子：

```C++
#include<iostream>
#include<algorithm>
using namespace std;
bool cmp(int a,int b);
main(){
//sort函数第三个参数自己定义，实现从大到小
int a[]={45,12,34,77,90,11,2,4,5,55};
sort(a,a+10,cmp);
for(int i=0;i<10;i++)
cout<<a[i]<<" ";
}
//自定义函数
bool cmp(int a,int b){
return a>b;//a>b表示返回的是降序，值大的排在前面
//想要返回升序，值小的在前面，应返回：return a<b;
}
```

## pair类型或者结构体类型的sort函数排序

例子：

```C++
#include<iostream>
#include<algorithm>
#include"cstring"
using namespace std;

typedef struct student{
char name[20];
int math;
int english;
}Student;

bool cmp(Student a,Student b)
{
if(a.math==b.math )
return a.english>b.english//math相等，按endlish从大到小排序
else
return a.math <b.math ;//按math从小到大排序 
};
int main(){
Student a[4]={{"apple",67,89},{"limei",90,56},{"apple",90,99}};
sort(a,a+3,cmp);
}

```

## 自定义lamda函数做比较函数

例子

```C++
#include<iostream>
#include<algorithm>
#include<vector>

using namespace std;

int main()
{
    int n,m;
    cin>>n>>m;
    vector<pair<int,int> > orders;
    pair<int,int> order;
    int v,w;
    for(int i=0;i<n;i++)
    {
        cin>>v>>w;
        order.first=v+w*2;
        order.second=i+1;
        orders.push_back(order);
    }
    sort(orders.begin(),orders.end(),[](pair<int,int>&a,pair<int,int>&b)
    {
        if(a.first==b.first)
        return a.second<b.second;//第一个值相等的话，将第二个值升序，从小到大
        else
        return a.first>b.first;//对第一个值降序，从大到小
    });//利用Lamda函数作为比较函数。也可用作其他类型

```



