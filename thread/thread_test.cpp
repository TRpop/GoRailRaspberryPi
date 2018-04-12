#include<thread> 
#include<iostream> 

using namespace std; 

void func1() { 
	cout<<"my func1"<<endl; 
}
 
int main() { 
	thread t(&func1); //방법 1 
	t.join(); //join()을 실행시키면 t가 종료되기 전까지 기다린다. 
	return 0; 
}
