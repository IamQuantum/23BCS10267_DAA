#include<iostream>
using namespace std;

// Doubly Linked List Node
class DoublyNode {
public:
    int data;
    DoublyNode* next;
    DoublyNode* prev;

    DoublyNode(int value) {
        data = value;
        next = nullptr;
        prev = nullptr;
    }
};

// Doubly Linked List
class DoublyLinkedList {
    DoublyNode* head;
    DoublyNode* tail;

public:
    DoublyLinkedList() {
        head = nullptr;
        tail = nullptr;
    }

    ~DoublyLinkedList() {
        while (head) {
            DoublyNode* temp = head;
            head = head->next;
            delete temp;
        }
    }

    void insertAtBeginning(int value) {
        DoublyNode* newNode = new DoublyNode(value);
        if (!head) {
            head = tail = newNode;
        } else {
            newNode->next = head;
            head->prev = newNode;
            head = newNode;
        }
    }

    void insertAtEnd(int value) {
        DoublyNode* newNode = new DoublyNode(value);
        if (!tail) {
            head = tail = newNode;
        } else {
            tail->next = newNode;
            newNode->prev = tail;
            tail = newNode;
        }
    }

    void deleteAtBeginning() {
        if (!head) return;
        DoublyNode* temp = head;
        if (head == tail) {
            head = tail = nullptr;
        } else {
            head = head->next;
            head->prev = nullptr;
        }
        delete temp;
    }

    void deleteAtEnd() {
        if (!tail) return;
        DoublyNode* temp = tail;
        if (head == tail) {
            head = tail = nullptr;
        } else {
            tail = tail->prev;
            tail->next = nullptr;
        }
        delete temp;
    }

    void traverse() {
        DoublyNode* current = head;
        while (current) {
            cout << current->data << " ";
            current = current->next;
        }
        cout << endl;
    }
};

// Circular Linked List Node
class CircularNode {
public:
    int data;
    CircularNode* next;

    CircularNode(int value) {
        data = value;
        next = nullptr;
    }
};

// Circular Linked List
class CircularLinkedList {
    CircularNode* head;

public:
    CircularLinkedList() {
        head = nullptr;
    }

    ~CircularLinkedList() {
        if (!head) return;
        CircularNode* current = head;
        do {
            CircularNode* temp = current;
            current = current->next;
            delete temp;
        } while (current != head);
    }

    void insertAtBeginning(int value) {
        CircularNode* newNode = new CircularNode(value);
        if (!head) {
            head = newNode;
            newNode->next = head;
        } else {
            CircularNode* tail = head;
            while (tail->next != head) {
                tail = tail->next;
            }
            newNode->next = head;
            tail->next = newNode;
            head = newNode;
        }
    }

    void insertAtEnd(int value) {
        CircularNode* newNode = new CircularNode(value);
        if (!head) {
            head = newNode;
            newNode->next = head;
        } else {
            CircularNode* tail = head;
            while (tail->next != head) {
                tail = tail->next;
            }
            tail->next = newNode;
            newNode->next = head;
        }
    }

    void deleteAtBeginning() {
        if (!head) return;
        if (head->next == head) {
            delete head;
            head = nullptr;
        } else {
            CircularNode* tail = head;
            while (tail->next != head) {
                tail = tail->next;
            }
            CircularNode* temp = head;
            head = head->next;
            tail->next = head;
            delete temp;
        }
    }

    void deleteAtEnd() {
        if (!head) return;
        if (head->next == head) {
            delete head;
            head = nullptr;
        } else {
            CircularNode* current = head;
            while (current->next->next != head) {
                current = current->next;
            }
            CircularNode* temp = current->next;
            current->next = head;
            delete temp;
        }
    }

    void traverse() {
        if (!head) return;
        CircularNode* current = head;
        do {
            cout << current->data << " ";
            current = current->next;
        } while (current != head);
        cout << endl;
    }
};

int main() {
    // Doubly Linked List Demo
    cout << "Doubly Linked List:" << endl;
    DoublyLinkedList dll;
    dll.insertAtBeginning(10);
    dll.insertAtBeginning(20);
    dll.insertAtEnd(30);
    dll.insertAtEnd(40);
    dll.traverse(); // 20 10 30 40
    
    dll.deleteAtBeginning();
    dll.deleteAtEnd();
    dll.traverse(); // 10 30

    // Circular Linked List Demo
    cout << "\nCircular Linked List:" << endl;
    CircularLinkedList cll;
    cll.insertAtBeginning(10);
    cll.insertAtBeginning(20);
    cll.insertAtEnd(30);
    cll.insertAtEnd(40);
    cll.traverse(); // 20 10 30 40
    
    cll.deleteAtBeginning();
    cll.deleteAtEnd();
    cll.traverse(); // 10 30

    return 0;
}