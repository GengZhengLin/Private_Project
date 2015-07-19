#include <windows.h>
#include <iostream>
using namespace std;

typedef void (CALLBACK* func1)();
typedef char* (CALLBACK* func2)();
int main()
{
	HINSTANCE hDLL;
	func1 API_Init;
	func2 API_Update_Frame;
	hDLL = LoadLibrary(L"game_dll");
	if (!hDLL)
	{
		cout << "dll not found!" << endl;
		return 0;
	}
	API_Init = (func1)GetProcAddress(hDLL, "API_Init");
	API_Update_Frame = (func2)GetProcAddress(hDLL, "API_Update_Frame");
	if (!API_Init)
	{
		cout << "API_Init not found!" << endl;
		return 0;
	}
	if (!API_Update_Frame)
	{
		cout << "API_Update_Frame not found!" << endl;
		return 0;
	}


	API_Init();
	while (true)
	{
		cout << API_Update_Frame() << endl;
	}
}