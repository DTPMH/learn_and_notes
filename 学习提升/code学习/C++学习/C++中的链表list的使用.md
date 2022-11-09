<center><span style="font-size:2rem;font-weight:bold;">C++中的链表库list的使用</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 简介与头文件

list是支持常数时间从容器任何位置插入和移除元素的容器

头文件`#include<list>`

## 特点

* list是可以在常数范围内在任意位置进行插入和删除的序列式容器，并且该容器可以前后双向迭代。

* list的底层是[双向链表](https://so.csdn.net/so/search?q=双向链表&spm=1001.2101.3001.7020)结构，双向链表中每个元素存储在互不相关的独立节点中，在节点中通过指针指向其前一个元素和后一个元素。

* list 与forward_list非常相似：主要的不同在于forword_list是单链表，只能朝前迭代。

* 与其他的序列是容器相比（array，vector，deque）list 通常在任意位置进行插入、移除元素的效率更高。

* 与其他序列是容器相比，list 和 forward_list最大的缺陷是不支持任意位置的随机访问，

  比如：要访问list 的第6个元素，必须从已知位置（比如头部或者尾部）迭代到该位置，在这段位置上迭代需要线性的时间开销；list 还需要一些额外的空间，以保存每个结点的相关信息。

# 定义与初始化

```C++
std::list<std::string> words1 {"the", "frogurt", "is", "also", "cursed"};
std::list<std::string> words2(words1.begin(), words1.end());
std::list<std::string> words3(words1);
// words4 是 {"Mo", "Mo", "Mo", "Mo", "Mo"}
 std::list<std::string> words4(5, "Mo");
```

# assign函数

以副本的元素替换原本内容

```C++
std::list<char> characters;
characters.assign(5, 'a');
characters.assign({'\n', 'C', '+', '+', '1', '1', '\n'});
characters2.assign(characters.begin(), characters.end());
```

# 元素访问

```C++
list.front();
list.back();
std::list<int>::iterator it;
it=list.begin();
cout<<*it<<endl;
```

# 元素遍历

```C++
list.begin();        //返回指向起始的迭代器 
list.end();          //返回指向末尾的迭代器 
list.rbegin();       //返回指向起始的逆向迭代器 
list.rend();         //返回指向末尾的逆向迭代器 
// 移除所有偶数
for (std::list<int>::iterator it = c.begin(); it != c.end();)
{
    if (*it % 2 == 0)
        it = c.erase(it);
    else
        ++it;
}
```

# 容量

```C++
list.empty();
list.size();
list.max_size();
```

# 插入元素

## insert函数

```C++
std::list<int> c1(3, 100);
auto it = c1.begin();
it = c1.insert(it, 200);         //在起始处插入元素200
c1.insert(it, 2, 300);           //在起始处插入2个元素300
// 将 it 重新指向开头
it = c1.begin();
std::list<int> c2(2, 400);
c1.insert(std::next(it, 2), c2.begin(), c2.end());   //在c1列表中的第二个元素的下一个元素(即第三个元素)插入列表c2

int arr[] = {501, 502, 503};
c1.insert(c1.begin(), arr, arr + std::size(arr));   //在起始处插入数组arr

c1.insert(c1.end(), {601, 602, 603});     //在末尾插入初始化列表
```

## empalce函数

```C++
emplace( const_iterator pos, Args&&... args )             //模板参数
例子
list.emplace(list.end(),300);
```

## empalce_back函数

在容器末尾就地构造元素 

```C++
list.emplace_back(300);
```

## push_back函数

将元素添加到容器末尾 

```C++
list.push_back(300);
```

## push_front函数

插入元素到容器起始 

```C++
list.push_front(300);
```

## emplace_front函数

在容器头部原位构造元素 

```C++
list.emplace_front(300);
```

# 移除元素

```C++
list.pop_back();          //删除末尾元素
list.pop_front();         //删除起始元素

std::list<int> c{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
c.erase(c.begin());       //删除起始元素
std::list<int>::iterator range_begin = c.begin();
std::list<int>::iterator range_end = c.begin();
c.erase(c.begin(), c.end());   //删除所有
```

# 排序函数

```C++
 
std::ostream& operator<<(std::ostream& ostr, const std::list<int>& list)
{
    for (auto &i : list) {
        ostr << " " << i;
    }
    return ostr;
}
 
int main()
{
    std::list<int> list = { 8,7,5,9,0,1,3,2,6,4 };
 
    std::cout << "before:     " << list << "\n";
    list.sort();
    std::cout << "ascending:  " << list << "\n";
    list.sort(std::greater<int>());
    std::cout << "descending: " << list << "\n";
}
```

输出

```C++
before:      8 7 5 9 0 1 3 2 6 4
ascending:   0 1 2 3 4 5 6 7 8 9
descending:  9 8 7 6 5 4 3 2 1 0
```

