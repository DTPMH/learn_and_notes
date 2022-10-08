<center><span style="font-size:2rem;font-weight:bold;">C++位运算定义与常用使用技巧</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# C++中的位运算符

## 左移运算符：`<<`

左移运算符：将一个运算对象的各个二进制位全部左移若干位（左边的二进制位丢弃，右边补0）

**左移运算符的使用技巧：**

```C++
A<<1;                   //返回AX2
a<<2;                   //返回 ax2^2
```

## 右移运算符：`>>`

右移运算符：将一个数的二进制位全部右移若干位，正数左补0，负数左补1，右边丢弃

**右移运算符的使用技巧：**

```C++
A<<1;                   //返回A/2
a<<2;                   //返回 a/2^2
```

## 按位与运算符：`&`

运算规则：0&0=0；0&1=0；1&0=0；1&1=1；

### **按位与运算符使用技巧**

* 判断整数奇数偶数

```C++
if(a&1==0) return 偶数;
if(a&1==1) return 奇数；
```

* 清零

```C++
如果想将一个单元清零，只要与一个各位都为0的数值相与，结果为0;
```

* 求解m的n次方

```C++
#include<bits/stdc++.h>
using namespace std;
int main()
{
    //求3的11次
    int m = 3,n=11;  //11的二进制为 1011
    //m^11 = m^1000 * m^0010 * m^0001;
    int sum = 1;
    while (n != 0) 
    {
        if (n & 1 == 1) //判断最后一位是不是1 是的话就乘以
            sum *= m;
        m *= m;
        n = n >> 1; //右移一位
    }
    cout << sum << endl;
    return 0;
}
```

* 判断整数m是否为2的整数幂

如果一个整数是2的整数次方，那么它的二进制标识中一定有且只有一位是1，而其他所有位均为0.

解决方案：

把这个整数减去1之后再和本身做与运算，这个整数中唯一的一个1就会变成0.所以只要判断是不是等于0即可。

```C++
#include<bits/stdc++.h>
using namespace std;
int main()
{
	int a = 64 & 63;// 01000000 & 00111111 = 0   64是2的整数幂
	int b = 8 & 7;// 1000 & 0111 = 0     8是2的整数幂
	int c = 7 & 6;// 0111 & 0110 = 6     7不是2的整数幂
	int n;
	cin >> n;
	if (n & (n - 1))
		cout << "NO\n";
	else
		cout << "Yes\n";
}
```

## 按位或运算符：`|`

按位或运算符：参加运算的两个对象，按二进制位进行“或”运算。

运算规则：0|0=0；0|1=1；1|0=1；1|1=1;

例如：3（0000 0011）|5（0000 0101）=（0000 0111）=7

**使用技巧**

*  对一个数据的某些位 置1

```C++
例如：X=1010 0000的低4为置1：X|0000 1111=1010 1111
```

## 异或运算符`^`

 参加运算的两个数，如果两个相应位为“异”（值不同），则该位结果为1，否则为0。

运算规则：0^0=0；0^1=1；1^0=1；1^1=0;

### 计算规则

* 交换律

```C++
a ^ b = b ^ a
```

* 恒等率

```c++
a ^ 0 = a
```

* 归零率

```C++
 a ^ a = 0
```

* 结合律

```C++
a ^ a = 0
```

### 使用技巧

* 交换两个数

```C++
#include<bits/stdc++.h>
using namespace std;
int main()
{
    int a,b;//a = 3 = 0011  b = 4 = 0100
    while (cin >> a >> b)
    {
        a = a ^ b; //a = 0011 ^ 0100 = 0111
        b = a ^ b; //b = 0111 ^ 0100 = 0011
        a = a ^ b; //a = 0111 ^ 0011 = 0100
        cout << a << " " << b;
    }
    return 0;
}
```

* 找出只出现一次的整数

```C++
#include<bits/stdc++.h>
using namespace std;
int main()
{
	int n, a[100],q;
	while (cin >> n)
	{
		q = 0;
		for (int i = 0;i < n;i++)
		{
			cin >> a[i];
			q ^= a[i];
		}
		cout << q << endl;
	}
	return 0;
}
```

## 取反运算符

~1=0；~0=1；

# bitset的用法

## 头文件

C++的 bitset 在 bitset 头文件中，它是一种类似数组的结构，它的每一个元素只能是０或１，每个元素仅用１bit空间。

## 构造函数

```C++
bitset<4> bitset1;　　    //无参构造，长度为４，默认每一位为０
bitset<8> bitset2(12);　　//长度为８，二进制保存，前面用０补充
string s = "100101";
bitset<10> bitset3(s);　　//长度为10，前面用０补充
char s2[] = "10101";
bitset<13> bitset4(s2);　　//长度为13，前面用０补充

cout << bitset1 << endl;　　//0000
cout << bitset2 << endl;　　//00001100
cout << bitset4 << endl;　　//0000000010101
```

**注意：**用字符串构造时，字符串只能包含 '0' 或 '1' ，否则会抛出异常。

构造时，需在<>中表明bitset 的大小(即size)。

在进行有参构造时，若参数的二进制表示比bitset的size小，则在前面用０补充(如上面的例子)；若比bitsize大，**参数为整数时取后面部分，参数为字符串时取前面部分(如下面例子)：**

```C++
bitset<2> bitset1(12);　　//12的二进制为1100（长度为４），但bitset1的size=2，只取后面部分，即00
string s = "100101";　　
bitset<4> bitset2(s);　　//s的size=6，而bitset的size=4，只取前面部分，即1001
char s2[] = "11101";
bitset<4> bitset3(s2);　　//与bitset2同理，只取前面部分，即1110

cout << bitset1 << endl;　　//00
cout << bitset2 << endl;　　//1001
cout << bitset3 << endl;　　//1110
```

## 可用的操作符

```C++
    bitset<4> foo (string("1001"));
    bitset<4> bar (string("0011"));

    cout << (foo^=bar) << endl;       // 1010 (foo对bar按位异或后赋值给foo)
    cout << (foo&=bar) << endl;       // 0010 (按位与后赋值给foo)
    cout << (foo|=bar) << endl;       // 0011 (按位或后赋值给foo)

    cout << (foo<<=2) << endl;        // 1100 (左移２位，低位补０，有自身赋值)
    cout << (foo>>=1) << endl;        // 0110 (右移１位，高位补０，有自身赋值)

    cout << (~bar) << endl;           // 1100 (按位取反)
    cout << (bar<<1) << endl;         // 0110 (左移，不赋值)
    cout << (bar>>1) << endl;         // 0001 (右移，不赋值)

    cout << (foo==bar) << endl;       // false (0110==0011为false)
    cout << (foo!=bar) << endl;       // true  (0110!=0011为true)

    cout << (foo&bar) << endl;        // 0010 (按位与，不赋值)
    cout << (foo|bar) << endl;        // 0111 (按位或，不赋值)
    cout << (foo^bar) << endl;        // 0101 (按位异或，不赋值)
```

## 元素读取

```C++
    bitset<4> foo ("1011");
    
    cout << foo[0] << endl;　　//1
    cout << foo[1] << endl;　　//1
    cout << foo[2] << endl;　　//0
```

## 可用函数

```C++
    bitset<8> foo ("10011011");

    cout << foo.count() << endl;　　//5　　（count函数用来求bitset中1的位数，foo中共有５个１
    cout << foo.size() << endl;　　 //8　　（size函数用来求bitset的大小，一共有８位

    cout << foo.test(0) << endl;　　//true　　（test函数用来查下标处的元素是０还是１，并返回false或true，此处foo[0]为１，返回true
    cout << foo.test(2) << endl;　　//false　　（同理，foo[2]为０，返回false

    cout << foo.any() << endl;　　//true　　（any函数检查bitset中是否有１
    cout << foo.none() << endl;　　//false　　（none函数检查bitset中是否没有１
    cout << foo.all() << endl;　　//false　　（all函数检查bitset中是全部为１
```

**注：test函数会对下标越界作出检查，而通过 [ ] 访问元素却不会经过下标检查，所以，在两种方式通用的情况下，选择test函数更安全一些**

## 其他函数

```C++
    bitset<8> foo ("10011011");

    cout << foo.flip(2) << endl;　　//10011111　　（flip函数传参数时，用于将参数位取反，本行代码将foo下标２处"反转"，即０变１，１变０
    cout << foo.flip() << endl;　　 //01100000　　（flip函数不指定参数时，将bitset每一位全部取反

    cout << foo.set() << endl;　　　　//11111111　　（set函数不指定参数时，将bitset的每一位全部置为１
    cout << foo.set(3,0) << endl;　　//11110111　　（set函数指定两位参数时，将第一参数位的元素置为第二参数的值，本行对foo的操作相当于foo[3]=0
    cout << foo.set(3) << endl;　　  //11111111　　（set函数只有一个参数时，将参数下标处置为１

    cout << foo.reset(4) << endl;　　//11101111　　（reset函数传一个参数时将参数下标处置为０
    cout << foo.reset() << endl;　　 //00000000　　（reset函数不传参数时将bitset的每一位全部置为０
```

## 类型转换的函数

```C++
    bitset<8> foo ("10011011");

    string s = foo.to_string();　　//将bitset转换成string类型
    unsigned long a = foo.to_ulong();　　//将bitset转换成unsigned long类型
    unsigned long long b = foo.to_ullong();　　//将bitset转换成unsigned long long类型

    cout << s << endl;　　//10011011
    cout << a << endl;　　//155
    cout << b << endl;　　//155
```

