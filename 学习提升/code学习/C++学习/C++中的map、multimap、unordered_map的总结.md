<center><span style="font-size:2rem;font-weight:bold;">C++中的map、multimap、unordered_map的总结</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# map的使用

`std::map` 是有序键值对容器，它的元素的键是唯一的。用比较函数 `Compare` 排序键。搜索、移除和插入操作拥有对数复杂度。 map 通常实现为[红黑树](https://en.wikipedia.org/wiki/Red–black_tree)

Map是C++的一个标准容器，他提供了很好的一对一的关系，在一些程序中建立一个map可以起到事半功倍的效果，总结了一些基本简单实用的操作

第一个可以成为关键字（key）,每个关键字只能在map中出现一次；

第二个可能称为该关键字的值（value）；

使用map得包含map类所在的头文件#include<map>//注意，STL头文件没有扩展名.h

## 定义与初始化

```C++
#include <iostream>
#include <string>
#include <iomanip>
#include <map>
 
template<typename Map>
void print_map(Map& m)
{
   std::cout << '{';
   for(auto& p: m)
        std::cout << p.first << ':' << p.second << ' ';
   std::cout << "}\n";
}
 
struct Point { double x, y; };
struct PointCmp {
    bool operator()(const Point& lhs, const Point& rhs) const { 
        return lhs.x < rhs.x; // NB 。有意忽略 y
    }
};
 
int main()
{
  // (1) 默认构造函数
  std::map<std::string, int> map1;
  map1["something"] = 69;
  map1["anything"] = 199;
  map1["that thing"] = 50;
  std::cout << "map1 = "; print_map(map1);
 
  // (2) 范围构造函数
  std::map<std::string, int> iter(map1.find("anything"), map1.end());
  std::cout << "\niter = "; print_map(iter);
  std::cout << "map1 = "; print_map(map1);
 
  // (3) 复制构造函数
  std::map<std::string, int> copied(map1);
  std::cout << "\ncopied = "; print_map(copied);
  std::cout << "map1 = "; print_map(map1);
 
  // (4) 移动构造函数
  std::map<std::string, int> moved(std::move(map1));
  std::cout << "\nmoved = "; print_map(moved);
  std::cout << "map1 = "; print_map(map1);
 
  // (5) initializer_list 构造函数
  const std::map<std::string, int> init {
    {"this", 100},
    {"can", 100},
    {"be", 100},
    {"const", 100},
  };
  std::cout << "\ninit = "; print_map(init);
 
 
  // 定制关键类选项 1 ：
  // 使用比较 struct
  std::map<Point, double, PointCmp> mag = {
      { {5, -12}, 13 },
      { {3, 4},   5 },
      { {-8, -15}, 17 }
  };
 
  for(auto p : mag)
      std::cout << "The magnitude of (" << p.first.x
                << ", " << p.first.y << ") is "
                << p.second << '\n';
 
  // 定制关键类选项 2 ：
  // 使用比较 lambda
  // 此 lambda 按照其模比较点，注意其中模取自局部变量 mag
  auto cmpLambda = [&mag](const Point &lhs, const Point &rhs) { return mag[lhs] < mag[rhs]; };
  // 你亦可使用不依赖局部变量的 lambda ，像这样：
  // auto cmpLambda = [](const Point &lhs, const Point &rhs) { return lhs.y < rhs.y; };
  std::map<Point, double, decltype(cmpLambda)> magy(cmpLambda);
 
  // 各种插入元素的方式：
  magy.insert(std::pair<Point, double>({5, -12}, 13));
  magy.insert({ {3, 4}, 5});
  magy.insert({Point{-8.0, -15.0}, 17});
 
  std::cout << '\n';
  for(auto p : magy)
      std::cout << "The magnitude of (" << p.first.x
                << ", " << p.first.y << ") is "
                << p.second << '\n';
}
输出：

map1 = {anything:199 something:69 that thing:50 }
 
iter = {anything:199 something:69 that thing:50 }
map1 = {anything:199 something:69 that thing:50 }
 
copied = {anything:199 something:69 that thing:50 }
map1 = {anything:199 something:69 that thing:50 }
 
moved = {anything:199 something:69 that thing:50 }
map1 = {}
 
init = {be:100 can:100 const:100 this:100 }
The magnitude of (-8, -15) is 17
The magnitude of (3, 4) is 5
The magnitude of (5, -12) is 13
 
The magnitude of (3, 4) is 5
The magnitude of (5, -12) is 13
The magnitude of (-8, -15) is 17
```

## 元素访问

```C++
map.at(key);
map[key];
```

例子

```C++
#include <iostream>
#include <string>
#include <vector>
#include <map>
 
int main()
{
    std::map<char, int> letter_counts {{'a', 27}, {'b', 3}, {'c', 1}};
 
    std::cout << "initially:\n";
    for (const auto &pair : letter_counts) {
        std::cout << pair.first << ": " << pair.second << '\n';
    }
 
    letter_counts['b'] = 42;  // 更新既存值
    letter_counts['x'] = 9;  // 插入新值
 
    std::cout << "after modifications:\n";
    for (const auto &pair : letter_counts) {
        std::cout << pair.first << ": " << pair.second << '\n';
    }
 
    // 统计每个词的出现数
    // （首次调用 operator[] 以零初始化计数器）
    std::map<std::string, size_t>  word_map;
    for (const auto &w : { "this", "sentence", "is", "not", "a", "sentence",
                           "this", "sentence", "is", "a", "hoax"}) {
        ++word_map[w];
    }
 
    for (const auto &pair : word_map) {
        std::cout << pair.second << " occurrences of word '" << pair.first << "'\n";
    }
}
输出：

initially:
a: 27
b: 3
c: 1
after modifications:
a: 27
b: 42
c: 1
x: 9
2 occurrences of word 'a'
1 occurrences of word 'hoax'
2 occurrences of word 'is'
1 occurrences of word 'not'
3 occurrences of word 'sentence'
2 occurrences of word 'this'
```

## 迭代器

```C++
map.begin();         //返回指向起始的迭代器 
map.rbegin();        //返回指向起始的逆向迭代器 
map.cbegin();        //返回指向起始的迭代器 
map.end();           //返回指向末尾的迭代器 
map.rend();          //返回指向末尾的逆向迭代器 
map.cend();          //返回指向末尾的迭代器 
```

## 容量

```C++
map.size();
map.empty();
```

## 查找

```C++
map.count(key);        //返回key元素数量
map.find(key);         //找到返回迭代器，没找到返回end();
```

## 插入元素

```C++
map<int ,string >maplive;
maplive.insert(pair< int ,string >( 102, “aclive”));
maplive.insert(map< int ,string >::value_type(321, “hai”));
maplive[112]= “April”;//map中最基础的插入函数。112表示的关键字，等号后面即键值
//注：前两种插入方式，不能覆盖map中原本的键（key），而第三种插入方式可以覆盖原有的key
```

## 删除元素

```C++
map.erase(key);
map.erase(iterator);
map.erase(begin(),end());
```

```C++
例子：删除键值(key)为112的数据
map< int ,string >::iterator l_it;
l_it=maplive.find(112);
if(l_it==maplive.end())
cout<< “we do not find 112”<<endl;
else 
maplive.erase(l_it);
```

## 排序

与set排序一致，只能给key排序

# multimap 的使用

multimap与map不一致的地方就是可以插入重复key值元素

其他操作一致



# unordered_map的使用

unordered_map是一个关联容器，内部采用的是hash表结构，拥有快速检索的功能。

头文件：#include<unordered_map>

## 特性

关联性：通过key去检索value,而不是通过绝对地址

无序性：使用hash表存储，内部无序。

Map：每个值对用一个键值。

键唯一性：不存在两个元素的键不一样

动态内存管理：使用内存管理模型来动态管理所需要的内存空间。

## 常用函数

### 构造函数

```C++
#include <iostream>
#include <string>
#include <unordered_map>
using namespace std;

typedef unordered_map<string,string> stringmap;

stringmap merge (stringmap a,stringmap b) {
  stringmap temp(a); temp.insert(b.begin(),b.end()); return temp;
}

int main ()
{
  stringmap first;                              // 空
  stringmap second ( {{"apple","red"},{"lemon","yellow"}} );    // 用初始化列表初始化
  stringmap third ( {{"orange","orange"},{"strawberry","red"}} ); // 用初始化列表初始化
  stringmap fourth (second);                    // 复制初始化
  stringmap sixth (fifth.begin(),fifth.end());  // 范围初始化

  cout << "sixth contains:";
  for (auto& x: sixth) cout << " " << x.first << ":" << x.second;
  cout << endl;

  return 0;
}

输出结果：
sixth contains: apple:red lemon:yellow orange:orange strawberry:red
```

### 迭代器

unorderer_map迭代器是一个指针，指向这个元素，通过迭代器来取得它的值

```C++
map.begin();
map.end();
```

例子

```C++
#include <cmath>
#include <iostream>
#include <unordered_map>
 
struct Node { double x, y; };
 
int main() {
    Node nodes[3] = { {1, 0}, {2, 0}, {3, 0} };
 
    // mag 是映射 Node 地址到其平面中长度的 map
    std::unordered_map<Node *, double> mag = {
        { nodes,     1 },
        { nodes + 1, 2 },
        { nodes + 2, 3 }
    };
 
    // 从 0 到长度更改每个 y 坐标
    for(auto iter = mag.begin(); iter != mag.end(); ++iter){
        auto cur = iter->first; // 指向 Node 的指针
        cur->y = mag[cur]; // 亦能使用 cur->y = iter->second;
    }
 
    // 更新并打印每个结点的长度
    for(auto iter = mag.begin(); iter != mag.end(); ++iter){
        auto cur = iter->first;
        mag[cur] = std::hypot(cur->x, cur->y);
        std::cout << "The magnitude of (" << cur->x << ", " << cur->y << ") is ";
        std::cout << iter->second << '\n';
    }
 
    // 以基于范围的 for 循环重复上述内容
    for(auto i : mag) {
        auto cur = i.first;
        cur->y = i.second;
        mag[cur] = std::hypot(cur->x, cur->y);
        std::cout << "The magnitude of (" << cur->x << ", " << cur->y << ") is ";
        std::cout << mag[cur] << '\n';
        // 注意与上述 std::cout << iter->second << '\n'; 相反，
        // std::cout << i.second << '\n'; 将不会打印更新的长度
    }
}
可能的输出：

The magnitude of (3, 3) is 4.24264
The magnitude of (1, 1) is 1.41421
The magnitude of (2, 2) is 2.82843
The magnitude of (3, 4.24264) is 5.19615
The magnitude of (1, 1.41421) is 1.73205
The magnitude of (2, 2.82843) is 3.4641
```

### 容量

```C++
size():返回unordered_map的大小
empty():为空返回为true，不为空返回false
```

### 查找

```C++
map.find(key);     
查找key所在的元素：
找到：返回元素的迭代器。通过迭代器的second属性获取值
没找到：返回unordered_map::end
```

### 插入元素

```C++
pair<string,double> myshopping ("baking powder",0.3);
map.insert(myshopping);
map.insert (make_pair<string,double>("eggs",6.0));
map.insert ({{"sugar",0.8},{"salt",0.1}});    // 初始化列表插入(可以用二维一次插入多个元素，也可以用一维插入一个元素)
map["coffee"] = 10.0;  //数组形式插入
```

### 访问元素

```C++
map.at(key);
map[key];
```

### 删除元素

```C++
map.clear();
map.erase(key);
map.erase(iterator);
map.erase(begin(),end());
```