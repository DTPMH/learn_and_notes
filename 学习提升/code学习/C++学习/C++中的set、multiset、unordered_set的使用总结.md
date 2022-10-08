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
set.insert();
set.emplace();
set.emplce_hint();
```

比较

```C++
bool com(int l,int r){return l<r;} 
set<int, decltypr(com)>
```

