	<center><span style="font-size:2rem;font-weight:bold;">C++中数组的总结</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# C/C++数组的使用

C/C++原本就支持数组类型，因此在C++中使用数组也可以不使用标准库(STL)

## 数组定义与初始化

### 一维数组的定义与初始化

**定义形式如下：**

**类型说明符	数组名[常量表达式]**

```C++
int array[5];
int array[5] = {1,2,3,4,5};
int array[5] = {1,2,3};
int array[] = {1,2,3,4,5};
```

### 一维数组读取元素

**引用形式如下：**

**数组名[下标]**

```C++
array[1];
```

## 二维数组的定义与初始化

**定义形式如下：**

**类型说明符	数组名[常量表达式1] [常量表达式2]**

```C++
int array[2][3];
int array[2][3] = {{1,2,3},{2,3,4}};
int array[2][3] = {1,2,3,4,5,6};
int array[2][3] = {1,2,3};
int array[2][3] = {{1},{2,3}};
int array[][3] = {{1},{2,3}};
```

### 一维数组读取元素

**引用形式如下：**

**数组名[下标1] [下标2]**

```C++
array[1][2];
```

# STL标准库array的使用

## 简介与包含头文件

头文件：`#include<array>`

array是序列式容器，类似于C语言的数组，是固定大小的，一旦定义完成后就无法进行扩容或收缩

## 定义与初始化

```C++
std::array<double, 10> values;            //定义一个容量为10的double型数组，其中值不确定
std::array<double, 10> values {};         //定义一个容量为10的double型数组，其中值都为0
std::array<double, 10> values {0.5,1.0,1.5,2.0};           //定义一个容量为10的double型数组，根据花括号内容初始化数组，不足补0
```

## 常见用法

### 遍历元素

```C++
array.font();
array.back();
array.at(int pos);
arrar[int pos];
array.data();                     //返回数组指针，范围内
array.begin();
array.end();
array.rbegin();
array.rend();
```

### 容量函数

```C++
array.size();
array.empty();
array.max_size();
```

### 填充与交换函数

```C++
array.fill(78);              //想数组中填充78
array.swap(array2);          //交换两个数组中的元素
```

### 特殊函数

```C++
std::get<0>(arr);         //获取arr中位于0的元素，可读可写
std::get<0>(arr)=1；
std::swap(array1,array2);  //交换两个数组的元素
auto array = std::to_array<int>({1,2,3});
auto array = std::to_array<pair<int,float>>( { 3, .0f }, { 4, .1f }, { 4, .1e23f } });
```

例子

```C++
#include <type_traits>
#include <utility>
#include <array>
#include <memory>
 
int main()
{
    // 复制字符串字面量
    auto a1 = std::to_array("foo");
    static_assert(a1.size() == 4);
 
    // 推导元素类型和长度
    auto a2 = std::to_array({ 0, 2, 1, 3 });
    static_assert(std::is_same_v<decltype(a2), std::array<int, 4>>);
 
    // 推导长度而元素类型指定
    // 发生隐式转换
    auto a3 = std::to_array<long>({ 0, 1, 3 });
    static_assert(std::is_same_v<decltype(a3), std::array<long, 3>>);
 
    auto a4 = std::to_array<std::pair<int, float>>(
        { { 3, .0f }, { 4, .1f }, { 4, .1e23f } });
    static_assert(a4.size() == 3);
 
    // 创建不可复制的 std::array
    auto a5 = std::to_array({ std::make_unique<int>(3) });
    static_assert(a5.size() == 1);
 
    // 错误：不支持复制多维数组
    // char s[2][6] = { "nice", "thing" };
    // auto a6 = std::to_array(s);
}
```





