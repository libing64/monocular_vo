#include <iostream>
#include <list>

using namespace std;
int main()
{

    list<int> buf;
    for (int i = 0; i < 10; i++)
    {
        buf.push_back(i);
    }
    list<int>::iterator it;
    for (list<int>::iterator ii = buf.begin(); ii != buf.end(); ii++)
    {
        if (*ii == 5)
        {
            it = ii;
        }
    }

    cout << "it: " << *it << endl;
    cout << "buf size: " << buf.size() << endl;

    for (list<int>::iterator ii = buf.begin(); ii != buf.end(); ii++)
    {
        int val = *ii;
        if (val % 2 == 0)
        {
            ii = buf.erase(ii);
        }
    }

    cout << "it: " << *it << endl;
    cout << "buf size: " << buf.size() << endl;

    return 0;
}