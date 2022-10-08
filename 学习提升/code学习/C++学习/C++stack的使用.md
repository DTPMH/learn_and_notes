<center><span style="font-size:2rem;font-weight:bold;">C++stack堆栈的使用</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 简介与头文件

栈是基本的[数据结构](https://so.csdn.net/so/search?q=数据结构&spm=1001.2101.3001.7020)之一,特点是`先进后出`，即只能在栈顶进行操作。

头文件`#include<stack>`

# 定义与初始化

**注：创建堆栈时，不能在初始化列表中用对象来初始化，但是可以用另一个容器来初始化，只要堆栈的底层容器类型和这个容器的类型相同**

```C++
std::stack<int> c1;
std::stack<int> c2(c1);
std::deque<int> deq {3, 1, 4, 1, 5};
std::stack<int> c3(deq);

std::vector<std::string>
        v1{"1","2","3","4"},
        v2{"Ɐ","B","Ɔ","D","Ǝ"};

std::stack s1{std::move(v1)};
std::stack s2{std::move(v2)};
```

# 元素访问

```C++
stack.top();   //返回栈顶元素的引用
```

# 元素插入

```C++
stack.push(22);  //栈顶插入一个元素
stack.emplace(22);
```

# 元素删除

```C++
stack.pop();    //删除栈顶元素
```

# 元素交换

```C++
stack.swap(stack2);   //与stack2交换元素
```

