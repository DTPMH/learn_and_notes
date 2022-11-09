<center><span style="font-size:2rem;font-weight:bold;">C++中的set、multiset、unordered_set的总结</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# set的使用

## 头文件与简介

头文件`#include<iostream>`

set的底层是**红黑树**，是**不支持随机访问的类型**，迭代器只能++

## 特点

* set中不支持容器中有重复元素
* set中的元素是有序的
* set不支持随机访问，即不可以使用`at,[]`读取数据
* set中的迭代器不支持加数字，即`set.begin()+2`是错误的，只能通过iterator++的方式

## 定义与初始化

```C++
#include <iostream>
#include <string>
#include <set>
#include <cmath>
 
struct Point { double x, y; };
struct PointCmp {
    bool operator()(const Point& lhs, const Point& rhs) const { 
        return std::hypot(lhs.x, lhs.y) < std::hypot(rhs.x, rhs.y); 
    }
};
 
int main()
{
  // (1) 默认初始化器
  std::set<std::string> a;
  a.insert("cat");
  a.insert("dog");
  a.insert("horse");
  for(auto& str: a) std::cout << str << ' ';
  std::cout << '\n';
 
  // (2) 迭代器初始化器
  std::set<std::string> b(a.find("dog"), a.end());
  for(auto& str: b) std::cout << str << ' ';
  std::cout << '\n';
 
  // (3) 复制构造函数
  std::set<std::string> c(a);
  c.insert("another horse");
  for(auto& str: c) std::cout << str << ' ';
  std::cout << '\n';
 
  // (4) 移动构造函数
  std::set<std::string> d(std::move(a));
  for(auto& str: d) std::cout << str << ' ';
  std::cout << '\n';
  std::cout << "moved-from set is ";
  for(auto& str: a) std::cout << str << ' ';
  std::cout << '\n';
 
  // (5) initializer_list 构造函数
  std::set<std::string> e {"one", "two", "three", "five", "eight"};
  for(auto& str: e) std::cout << str << ' ';
  std::cout << '\n';
 
  // 自定义比较
  std::set<Point, PointCmp> z = {{2, 5}, {3, 4}, {1, 1}};
  z.insert({1, -1}); // 这会失败，因为 1,-1 的长度等于 1,1
  for(auto& p: z) std::cout << '(' << p.x << ',' << p.y << ") ";
  std::cout << '\n';
}
```

输出

```C++
cat dog horse 
dog horse 
another horse cat dog horse 
cat dog horse 
moved-from set is 
eight five one three two 
(1,1) (3,4) (2,5)
```

## 常用函数

### 迭代器

```C++
set.begin();         //返回指向起始的迭代器 
set.rbegin();        //返回指向起始的逆向迭代器 
set.cbegin();        //返回指向起始的迭代器 
set.end();           //返回指向末尾的迭代器 
set.rend();          //返回指向末尾的逆向迭代器 
set.cend();          //返回指向末尾的迭代器 
```

### 容量函数

```C++
set.empty();
set.size();
set.max_size();
```

### 插入元素

```C++
set.insert(key);
set.insert(a.begin(),a.end());
set.insert(std::initializer_list<value_type> ilist);
set.emplace();
set.emplce_hint();
```

### 删除元素

```C++
set.clear();
set.erase(iterator it);
set.erase(begin(),end());
set.erase(key);
```

例子

```C++
#include <set>
#include <iostream>
int main()
{
    std::set<int> c = { 1, 2, 3, 4, 1, 2, 3, 4 };
 
    auto print = [&c]
    {
        std::cout << "c = { ";
        for (int n : c)
            std::cout << n << ' ';
        std::cout << "}\n";
    };
    print();
 
    std::cout << "移除所有奇数：\n";
    for (auto it = c.begin(); it != c.end();)
    {
        if (*it % 2 != 0)
            it = c.erase(it);
        else
            ++it;
    }
    print();
 
    std::cout << "移除 1，移除个数：" << c.erase(1) << '\n';
    std::cout << "移除 2，移除个数：" << c.erase(2) << '\n';
    std::cout << "移除 2，移除个数：" << c.erase(2) << '\n';
    print();
}
输出：

c = { 1 2 3 4 }
移除所有奇数：
c = { 2 4 }
移除 1，移除个数：0
移除 2，移除个数：1
移除 2，移除个数：0
c = { 4 }
```

### 查找元素

```C++
set.find(key);                //存在则返回迭代器，不存在则返回set.end()
set.count(key);               //存在返回1，不存在返回0
set.contain(key);             //存在返回true，不存在返回false
```

## 自定义排序

### 自带的比较方式

set自带的比较方式是升序，即从小到大

例子

```C++
set<int> nums;
nums.insert({7,4,3,8,2});
for(auto it :nums){
    cout<<it<<endl;//2,3,4,7,8
}
```

### 重载比较函数的自定义比较方式

只能用于自定义类型的比较，比如class与struct

例子

```C++
#include<iostream>
#include<set>
using namespace std;
 
class Node
{
public:
	int start;
	int end;
	Node(){}
	Node(int s, int e) :start(s), end(e)
	{
	
	}
	bool operator<(const Node &other) const
	{
		return start < other.start;
	}
};
int main(void)
{
	multiset<Node> m;
	m.insert(Node(4, 5));
	m.insert(Node(3, 4));
	m.insert(Node(6, 7));
	system("pause");
	return 0;
}
```

### 重写仿函数自定义比较方式

例子

```C++
class mycompare
{
  public:
  bool operator()(int v1,int v2)//分别代表重载()和重载后参数列表

  {
    return v1>v2;
  }
};

set<int,mycompare> nums;
```

```C++
#include<iostream>
#include<set>
using namespace std;
 
class Node
{
public:
	int start;
	int end;
	Node(){}
	Node(int s, int e) :start(s), end(e)
	{
	
	}
};
 
class CompareClassNode
{
public:
	bool operator()(const Node &lhs, const Node &rhs)
	{
		return lhs.start < rhs.start;
	}
};
 
int main(void)
{
	multiset<Node,CompareClassNode> m;
	m.insert(Node(4, 5));
	m.insert(Node(3, 4));
	m.insert(Node(6, 7));
	system("pause");
	return 0;
}
```

### lamda函数或者自定义函数比较方式

```c++
必须这样定义multiset<Node, decltype(compareNode)*> m(compareNode)。代码如下：
```

例子

```C++
#include<iostream>
#include<set>
 
using namespace std;
 
class Node
{
public:
	int start;
	int end;
	Node(){}
	Node(int s, int e) :start(s), end(e)
	{
	
	}
};
 
bool compareNode(const Node &lhs,const Node &rhs)
{
	return lhs.start < rhs.start;
}
 
int main(void)
{
	multiset<Node, decltype(compareNode)*> m(compareNode);
    auto com = [](const Node &lhs,const Node &rhs){
        return lhs.start < rhs.start;
    }
    multiset<Node, decltype(com)*> m2(com);
	m.insert(Node(4, 5));
	m.insert(Node(3, 4));
	m.insert(Node(6, 7));
	system("pause");
	return 0;
}
```

# multiset的使用

multiset的使用方法与set基本一致，不相同的地方时multiset支持重复元素

## 定义于与初始化

```C++
#include <iostream>
#include <string>
#include <set>
#include <cmath>
 
struct Point { double x, y; };
struct PointCmp {
    bool operator()(const Point& lhs, const Point& rhs) const { 
        return std::hypot(lhs.x, lhs.y) < std::hypot(rhs.x, rhs.y); 
    }
};
 
int main()
{
  // (1) 默认初始化器
  std::multiset<std::string> a;
  a.insert("cat");
  a.insert("dog");
  a.insert("horse");
  for(auto& str: a) std::cout << str << ' ';
  std::cout << '\n';
 
  // (2) 迭代器初始化器
  std::multiset<std::string> b(a.find("dog"), a.end());
  for(auto& str: b) std::cout << str << ' ';
  std::cout << '\n';
 
  // (3) 复制构造函数
  std::multiset<std::string> c(a);
  c.insert("another horse");
  for(auto& str: c) std::cout << str << ' ';
  std::cout << '\n';
 
  // (4) 移动构造函数
  std::multiset<std::string> d(std::move(a));
  for(auto& str: d) std::cout << str << ' ';
  std::cout << '\n';
  std::cout << "moved-from set is ";
  for(auto& str: a) std::cout << str << ' ';
  std::cout << '\n';
 
  // (5) initializer_list 构造函数
  std::multiset<std::string> e {"one", "two", "three", "five", "eight"};
  for(auto& str: e) std::cout << str << ' ';
  std::cout << '\n';
}
```

## 常用函数

### 迭代器

```C++
multiset.begin();
multiset.cbegin();
multiset.rbegin();
multiset.end();
multiset.cend();
multiset.rend();
```

### 容量

```C++
multiset.empty();
multiset.size();
```

### 插入元素

```C++
multiset.insert(key);
multiset.insert(a.begin(),a.end());
multiset.insert(std::initializer_list<value_type> ilist);
multiset.emplace();
multiset.emplce_hint();
```

### 删除元素

```C++
set.clear();
set.erase(iterator it);
set.erase(begin(),end());
set.erase(key);
```

例子

```C++
#include <set>
#include <iostream>
int main()
{
    std::multiset<int> c = { 1, 2, 3, 4, 1, 2, 3, 4 };
 
    auto print = [&c]
    {
        std::cout << "c = { ";
        for (int n : c)
            std::cout << n << ' ';
        std::cout << "}\n";
    };
    print();
 
    std::cout << "移除所有奇数：\n";
    for (auto it = c.begin(); it != c.end();)
    {
        if (*it % 2 != 0)
            it = c.erase(it);
        else
            ++it;
    }
    print();
 
    std::cout << "移除 1，移除个数：" << c.erase(1) << '\n';
    std::cout << "移除 2，移除个数：" << c.erase(2) << '\n';
    std::cout << "移除 2，移除个数：" << c.erase(2) << '\n';
    print();
}
输出：

c = { 1 1 2 2 3 3 4 4 }
移除所有奇数：
c = { 2 2 4 4 }
移除 1，移除个数：0
移除 2，移除个数：2
移除 2，移除个数：0
c = { 4 4 }
```

### 查找元素

```C++
multiset.find(key);                //存在则返回迭代器，不存在则返回multiset.end()
multiset.count(key);               //存在返回个数，不存在返回0
multiset.contain(key);             //存在返回true，不存在返回false
```

# unordered_set 的使用

unordered_set与set和multiset不同的地方是其内部结构为hash表，方便查找，但不支持排序

其余操作与以上类似，不在赘述
