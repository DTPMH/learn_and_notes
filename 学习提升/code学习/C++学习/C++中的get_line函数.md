<center><span style="font-size:2rem;font-weight:bold;">C++中的get_line函数</span></center>

虽然可以使用 cin 和 >> 运算符来输入字符串，但它可能会导致一些需要注意的问题。

**当 cin 读取数据时，它会传递并忽略任何前导白色空格字符（空格、制表符或换行符）。**
一旦它接触到第一个非空格字符即开始阅读，当它读取到下一个空白字符时，它将停止读取。以下面的语句为例：
cin >> namel;
可以输入 "Mark" 或 "Twain"，但不能输入 "Mark Twain"，因为 cin 不能输入包含嵌入空格的字符串。
下面程序演示了这个问题：

```C++
// This program illustrates a problem that can occur if
    // cin is used to read character data into a string object.
    #include <iostream>
    #include <string> // Header file needed to use string objects
    using namespace std;
    int main()
    {
        string name;
        string city;
        cout << "Please enter your name: ";
        cin >> name;
        cout << "Enter the city you live in: ";
        cin >> city;
        cout << "Hello, " << name << endl;
        cout << "You live in " << city << endl;
        return 0;
    }
程序输出结果：
Please enter your name： John Doe
Enter the city you live in: Hello, John
You live in Doe
```

**请注意，在这个示例中，用户根本没有机会输入 city 城市名。因为在第一个输入语句中，当 cin 读取到 John 和 Doe 之间的空格时，它就会停止阅读，只存储 John 作为 name 的值。**
**在第二个输入语句中， cin 使用键盘缓冲区中找到的剩余字符，并存储 Doe 作为 city 的值。**



**为了解决这个问题，可以使用一个叫做 getline 的 C++ 函数。**
此函数可读取整行，包括前导和嵌入的空格，并将其存储在字符串对象中。
getline 函数如下所示：
**getline(cin, inputLine);**
其中 cin 是正在读取的输入流，而 inputLine 是接收输入字符串的 string 变量的名称。
下面的程序演示了 getline 函数的应用：

```c++
// This program illustrates using the getline function
//to read character data into a string object.
#include <iostream>
#include <string> // Header file needed to use string objects
using namespace std;

int main()
{
    string name;
    string city;
    cout << "Please enter your name: ";
    getline(cin, name);
    cout << "Enter the city you live in: ";
    getline(cin, city);
    cout << "Hello, " << name << endl;
    cout << "You live in " << city << endl;
    return 0;
}
```