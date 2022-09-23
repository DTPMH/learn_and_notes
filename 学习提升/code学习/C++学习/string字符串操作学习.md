<center><span style="font-size:2rem;font-weight:bold;">string字符串操作学习</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# 构造函数

用到字符串的一些函数需要用到头文件`<string>`

**注：字符串末尾存在`\0`**

```C++
string str;
string s(str);
string s(str,2,5);               //从str中从s[2]开始赋值5个字符到s中，即将str[2]-str[6]初始化s
string s("slslslsl"5);           //将c_char类型向量的前5个字符作为s的初值
string s(5,c);                   //将5个c字符作为s的初值
string s(str,2)；                //将str字符串的从2个元素开始到最后一个元素作为s的初值
```

例子：
```C++
    string str1;               //生成空字符串
    string str2("123456789");  //生成"1234456789"的复制品
    string str3("12345", 0, 3);//结果为"123"
    string str4("0123456", 5);  //结果为"01234"
    string str5(5, '1');       //结果为"11111"
    string str6(str2, 2);      //结果为"3456789"
```

# string 的大小和容量

```C++
string str;
str.size()==str.length();             //返回str中的字符个数
str.max_size();                       //返回str最多包含的字符串
str.capacity();                       //返回str最多能包含的最大字符数
```

# string之间的大小比较

```C++
string a="sssss";
a > "ssss"
字典序在前的字符越小
```

## compare函数

compare函数输出值如下：

* 相等输出0
* 大于输出1
* 小于输出-1

```C++
sting str("ssssss");
str.compare(a);                  //比较str和a的大小
str.compare(1,3,a);              //从str[1]开始获取3个字符，构成子串与a比较
str.compare(1,3,a,1,3);          //从str[1]开始获取3个字符，构成子串,从a[1]开始获取3个字符构成子串，两个子串比较大小
```

例子

```C++
#include <cassert>
#include <string>
#include <iostream>
 
int main()
{
    std::string batman{"Batman"};
    std::string superman{"Superman"};
    int compare_result{0};
 
    // 1) 与另一字符串比较
    compare_result = batman.compare(superman);
    std::cout << "1) " <<
    (
        compare_result < 0 ? "Batman 在 Superman 前面。\n" :
        compare_result > 0 ? "Superman 在 Batman 前面。\n" :
        "Superman 和 Batman 一样。\n"
    );
 
    // 2) 与另一字符串比较子串
    compare_result = batman.compare(3, 3, superman)
    std::cout << "2) " <<
    (
        compare_result < 0 ? "man 在 Superman 前面。\n" :
        compare_result > 0 ? "Superman 在 man 前面。\n" :
        "man 和 Superman 一样。\n"
    );
 
    // 3) 与另一子串比较子串
    compare_result = batman.compare(3, 3, superman, 5, 3)};
    std::cout << "3) " <<
    (
        compare_result < 0 ? "man 在 man 前面。\n" :
        compare_result > 0 ? "man 在 man 前面。\n" :
        "man 和 man 一样。\n"
    );
 
    // 与另一子串比较子串
    // 默认到为另一 string 的末尾
    assert(compare_result == batman.compare(3, 3, superman, 5));
 
    // 4) 与另一 char 指针比较
    compare_result = batman.compare("Superman")};
    std::cout << "4) " <<
    (
        compare_result < 0 ? "Batman 在 Superman 前面。\n" :
        compare_result > 0 ? "Superman 在 Batman 前面。\n" :
        "Superman 和 Batman 一样。\n"
    );
 
    // 5) 与另一 char 指针比较子串
    compare_result = batman.compare(3, 3, "Superman")};
    std::cout << "5) " <<
    (
        compare_result < 0 ? "man 在 Superman 前面。\n" :
        compare_result > 0 ? "Superman 在 man 前面。\n" :
        "man 和 Superman 一样。\n"
    );
 
    // 6) 与另一 char 指针子串比较子串
    compare_result = batman.compare(0, 3, "Superman", 5)};
    std::cout << "6) " <<
    (
        compare_result < 0 ? "Bat 在 Super 前面。\n" :
        compare_result > 0 ? "Super 在 Bat 前面。\n" :
        "Super 和 Bat 一样。\n"
    );
}
```

输出

```C++
Batman 在 Superman 前面。
Superman 在 man 前面。
man 和 man 一样。
Batman 在 Superman 前面。
Superman 在 man 前面。
Bat 在 Super 前面。
```

# 字符串的拼接

```C++
string str;
str.push_back('s');
str.pop_back();                     //删掉最后一个字符
str.insert(0,'s');                  //在str[0]处插入s
str.insert(0,3,'s');                //在str[0]出插入3个s
str.insert(0,'s')
str.insert(0,"string");             //在str[0]处插入字符串string
str.insert(0,"string",0,5)；        //在str[0]处插入字符串string[0-5]构成的子串
    
str.append("def");                //在str后面加入def字符串
str+="def";                       //在str后面加入def字符串
```

# erase函数

```C++
str.erase(iterator p);               //删除迭代器p所指向的元素
str.erase(iterator start,iterator last);              //删除迭代区间[start,last)上所有的字符
str.erase(0,3);                         //删除str[0]开始的3个元素
str.erase(0);                          //删除从str[0]开始的所有元素
```

例子：

```C++
#include <iostream>
#include <algorithm>
#include <string>
 
int main()
{
    std::string s = "This Is An Example";
    std::cout << "1) " << s << '\n';
 
    s.erase(7, 3); // 使用重载 (1) 擦除 " An"
    std::cout << "2) " << s << '\n';
 
    s.erase(std::find(s.begin(), s.end(), ' ')); 使用重载 (2) 擦除第一个 ' '
    std::cout << "3) " << s << '\n';
 
    s.erase(s.find(' ')); // 使用重载 (1) 切除从 ' ' 到字符串结尾的部分
    std::cout << "4) " << s << '\n';
 
    auto it = std::next(s.begin(), s.find('s')); // 获取指向第一个 's' 的迭代器
    s.erase(it, std::next(it, 2)); // 使用重载 (3) 擦除 "sI"
    std::cout << "5) " << s << '\n';
}
```

输出

```C++
1) This Is An Example
2) This Is Example
3) ThisIs Example
4) ThisIs
5) This
```

# clear函数

清空str的所有字符

# replace函数

```C++
string str;
str.replace(0,3,a);                      //将str中str[0]-str[2]的部分用字符串a替换
str.replace(str.begin(),str.begin()+2,a);          //将str中str[0]-str[2]的部分用字符串a替换
str.replace(0,3,a,1,2);                          //将str中str[0]-str[2]的部分用a字符串的a[1]-a[2]替换
str.replace(0,3,a,1,2);                          //将str中str[0]-str[2]的部分用a字符串的a[1]-a[2]替换
```

例子：

```C++
#include <iostream>
#include <string>
 
int main()
{
    std::string str("The quick brown fox jumps over the lazy dog.");
 
    str.replace(10, 5, "red"); // (5)
 
    str.replace(str.begin(), str.begin() + 3, 1, 'A'); // (6)
 
    std::cout << str << '\n';
}
```

输出

```C++
A quick red fox jumps over the lazy dog.
```

# find函数

```C++
str.find("ss",0);                 //从str[0]开始搜索ss字符串，并返回其所在位置（int）,如果没找到则返回：std::string::npos
str.find(str1,3);
```

## find_first_of函数

```C++
str.find_first_of("ss",0);         //从str[0]开始搜索第一次出现的ss字符串，并返回其所在位置（int）,如果没找到则返回：std::string::npos
str.find_first_of(str1,3);
```

## find_last_of函数

```C++
str.find_last_of("ss",0);         //从str[0]开始搜索最后一次出现的ss字符串，并返回其所在位置（int）,如果没找到则返回：std::string::npos
str.find_last_of(str1,3);
```

# substr子串提取函数

```C++
str.substr(0,3);              //返回str[0]-str[2]组成的子串
```

# copy函数

```C++
str.copy(dest,3,0);          //copy dest字符串的dest[0]-dest[2]赋值str,（dest,count,pos=0）
```

# 大小写转换

```C++
toupper('s');
tolower('S');
```

# to_string函数

将整形或者浮点型转换为string

```C++
to_string(7);
to_string(667.333);
```

# 字符串排序

```C++
sort(str.begin()，str.end());
```

# 字符串元素读取

```C++
str.at(4);
str.font();
str.back();
```

