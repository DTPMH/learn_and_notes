<center><span style="font-size:2rem;font-weight:bold;">C++中的队列(queue与priority_queue)用法详解</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# queue的使用详解

## 介绍

queue即数据结构中的队列，是一种特殊的线性表。队列是一个先进先出的(FIFO)的线性表，与栈相反(stack)。

queue在队尾插入，队头移除。如下所示：

![image-20220925125728993](C++中queue详解(and priority_queue).assets/image-20220925125728993.png)   



## queue 的定义与初始化

```C++
std::queue<int> c1;
std::queue<int> c2(c1);
std::deque<int> deq {3, 1, 4, 1, 5};
std::queue<int> c3(deq);
```

## 插入元素

```C++
std::queue<double> que;
que.push(87.0);
que.emplace(87.0);
```

## 遍历元素

queue每次智能获得队尾与队头的元素

```C++
std::queue<double> que;
que.front();
que.back();
```

## 删除元素

```C++
std::queue<double> que;
que.pop();
```

## 元素交换

```C++
que.swap(que2);
```

## 容量函数

```C++
que.empty(0);
que.size();
```



# 优先级队列priority_queue的使用

优先级队列是普通队列的升级，普通队列是一种先进先出的数据结构，元数在队列尾追加，而在队列头删除，在优先级队列中，元数被赋予优先级，当访问元素时，具有最高优先级的元素最先删除

## 构造函数

**priority_queue<Type,Container,Functional>，**
**其中Type为数据类型，Container为保存数据的容器，Functional元素比较方式**

```C++
例子：
priority_queue <int ,vector<int>, greater<int> > q;//升序队列，小顶堆（数值从小到大），与其他sort函数相反
priority_queue <int ,vector<int>, less<int> > q;//降序队列，大顶堆（数值从大到小）

priority_queue <int> a；//该声明相当于大顶堆
priority_queue <string> b;
```

**注：greater和less是std实现两个仿函数（就是使用一个类的使用看上去像一个函数，其实现就是在类中operator()，这个类就有类似函数的行为，就是一个仿函数类）。**

例子

```C++
#include <functional>
#include <queue>
#include <vector>
#include <iostream>
 
template<typename T> void print_queue(T& q) {
    while(!q.empty()) {
        std::cout << q.top() << " ";
        q.pop();
    }
    std::cout << '\n';
}
 
int main() {
    std::priority_queue<int> q;
 
    for(int n : {1,8,5,6,3,4,0,9,7,2})
        q.push(n);
 
    print_queue(q);
 
    std::priority_queue<int, std::vector<int>, std::greater<int> > q2;
 
    for(int n : {1,8,5,6,3,4,0,9,7,2})
        q2.push(n);
 
    print_queue(q2);
 
    // 用 lambda 比较元素。
    auto cmp = [](int left, int right) { return (left ^ 1) < (right ^ 1); };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> q3(cmp);
 
    for(int n : {1,8,5,6,3,4,0,9,7,2})
        q3.push(n);
 
    print_queue(q3);
 
}
```

输出

```C++
9 8 7 6 5 4 3 2 1 0 
0 1 2 3 4 5 6 7 8 9 
8 9 6 7 4 5 2 3 0 1
```

## 基本操作

```C++
que.top(); //访问队头元素
que.empty(0); //队列是否为空
que.size(); //返回队列内元素个数
que.push(90); //插入元素到队尾 (并排序)
que.emplace(90); //原地构造一个元素并插入队列
que.pop(); //弹出队头元素
que.swap(que2); //交换内容
```

## 排序详解

### 默认排序方式:降序

#### **默认的排序方式是降序，即从大到小排列**

```c++
#include <iostream>
#include <queue>
using namespace std;
int main()
{
    priority_queue<int> q;
    for( int i= 0; i< 10; ++i ) q.push(i);
    while( !q.empty() ){
        cout<<q.top()<<endl;
        q.pop();
    }
    return 0;
}

//cout: 9,8,7,6,5,4,3,2,1
```

**以上等同于：**

```C++
#include <iostream>
#include <queue>
using namespace std;
int main()
{
    priority_queue<int,vector<int>,less<int> > q;
    for( int i= 0; i< 10; ++i ) q.push(i);
    while( !q.empty() ){
        cout<<q.top()<<endl;
        q.pop();
    }
    return 0;
}

//cout: 9,8,7,6,5,4,3,2,1
```

#### 对pair类型的排序

利用pair的比较结果，先按照pair的first元素降序，first元素相等时，再按照second元素降序。

例子

```C++
#include<iostream>
#include<vector>
#include<queue>
using namespace std;
int main()
{
    priority_queue<pair<int,int> > coll;
    pair<int,int> a(3,4);
    pair<int,int> b(3,5);
    pair<int,int> c(4,3);
    coll.push(c);
    coll.push(b);
    coll.push(a);
    while(!coll.empty())
    {
        cout<<coll.top().first<<"\t"<<coll.top().second<<endl;
        coll.pop();
    }
    return 0;
}

//cout: 
//4,3
//3,5
//3,4
```

### 升序的排序方式

如果要用到小顶堆，则一般要模板的3个参数进去。STL里面定义了一个仿函数greater<>，基本类型可以用这个仿函数声明小顶堆。

#### 默认类型排序

例子

```C++
#include <iostream>
#include <queue> 
using namespace std;
int main(){
    priority_queue<int, vector<int>, greater<int> > q;
    for( int i= 0; i< 10; ++i ) q.push(10-i);
    while( !q.empty() ){
        cout << q.top() << endl;
        q.pop();
    }
    return 0;
}
```

#### 对pair类型的排序

利用pair作升序比较，先按照pair的first元素升序，first元素相等时，在按照second元素升序。

例子

```C++
#include<iostream>
#include<vector>
#include<queue>
using namespace std;
int main(){
    priority_queue<pair<int,int>,vector<pair<int,int> >,greater<pair<int,int> > > coll;
    pair<int,int> a(3,4);
    pair<int,int> b(3,5);
    pair<int,int> c(4,3);
    coll.push(c);
    coll.push(b);
    coll.push(a);
    while(!coll.empty())
    {
        cout<<coll.top().first<<"\t"<<coll.top().second<<endl;
        coll.pop();
    }
    return 0;
}
```

### 自定义的排序方式

#### 重载operator

例子

```C++
#include <iostream>
#include <queue> 
using namespace std;
struct Node{
    int x, y;
    Node(int a=0, int b=0):
        x(a),y(b){}
};
bool operator<(Node a, Node b){//返回true时，说明a的优先级低于b,与sort相反
    //x值较大的Node优先级低（x小的Node排在队前）
    //x相等时，y大的优先级低（y小的Node排在队前）
    if( a.x== b.x ) return a.y> b.y;
    return a.x> b.x; 
}
//这种排序方式为：先按x的值升序，后按y的值升序
int main(){
    priority_queue<Node> q;
    for( int i= 0; i< 10; ++i )
    q.push( Node( rand(), rand() ) );
    while( !q.empty() ){
        cout << q.top().x << ' ' << q.top().y << endl;
        q.pop();
    }
    return 0;
}
```

#### 重写comp仿函数

例子

```C++
#include <iostream>
#include <queue>
using namespace std;
struct Node{
    int x, y;
    Node( int a= 0, int b= 0 ):
        x(a), y(b) {}
};
struct cmp{
    bool operator() ( Node a, Node b ){//默认是less函数
        //返回true时，a的优先级低于b的优先级（a排在b的后面）
        if( a.x== b.x ) return a.y> b.y;      
        return a.x> b.x; }
};
int main(){
    priority_queue<Node, vector<Node>, cmp> q;
    for( int i= 0; i< 10; ++i )
    q.push( Node( rand(), rand() ) );
    while( !q.empty() ){
        cout << q.top().x << ' ' << q.top().y << endl;
        q.pop();
    }
    return 0;
}
```

#### 利用lamda函数比较

例子

```C++
#include <functional>
#include <queue>
#include <vector>
#include <iostream>
 
template<typename T> void print_queue(T& q) {
    while(!q.empty()) {
        std::cout << q.top() << " ";
        q.pop();
    }
    std::cout << '\n';
}
int main() {
    // 用 lambda 比较元素。
    auto cmp = [](int left, int right) { return (left ^ 1) < (right ^ 1); };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> q3(cmp);
 
    for(int n : {1,8,5,6,3,4,0,9,7,2})
        q3.push(n);
    print_queue(q3);
 
}
```

