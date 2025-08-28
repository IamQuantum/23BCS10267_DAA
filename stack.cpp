#include<iostream>
using namespace std;

template<typename T>
class stack{
    int size;
    T* arr;
    int top;
    
    public:
        stack(int size) : size(size), arr(new T[size]), top(-1) {}
        
        ~stack() {
            delete[] arr;
        }
        
        bool is_empty(){
            return top == -1;
        }
        
        bool is_full(){
            return top == size - 1;
        }
        
        void push(T val){
            if (is_full()){
                throw overflow_error("Stack Overflow");
            }
            arr[++top] = val;
        }
        
        T pop(){
            if (is_empty()){
                throw underflow_error("Stack Underflow");
            }
            return arr[top--];
        }
        
        T peek(){
            if (is_empty()){
                throw underflow_error("Stack is empty");
            }
            return arr[top];
        }
        
        void display_status(){
            cout << "Stack status: ";
            if (is_empty()) {
                cout << "Empty" << endl;
            } else if (is_full()) {
                cout << "Full, top element: " << peek() << endl;
            } else {
                cout << "Has elements, top element: " << peek() << endl;
            }
        }
};

int main(){
    stack<int> s(5);
    
    // Check initial status
    s.display_status();
    
    // Push operations
    s.push(5);
    s.push(2);
    s.push(8);
    s.display_status();
    
    // Pop operation
    cout << "Popped: " << s.pop() << endl;
    s.display_status();
    
    return 0;
}