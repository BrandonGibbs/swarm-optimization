#ifndef PQUEUE_H
#define PQUEUE_H

#include <iostream>
#include <vector>
#include <utility> // pair, make_pair
#include <cmath>   // INFINITY

#define PARENT(index) (index >> 1)
#define LEFT(index)   (index << 1)
#define RIGHT(index)  ((index << 1) + 1)

using namespace std;

template <typename T> class PQueue {
public:
  PQueue(){}

  void push(T value, float priority){
    heap.push_back(make_pair (value, INFINITY));
    decreasePriority(heap.size() - 1, value, priority);
  }

  T pop(){
    T value = heap.front().first; // get the value we want
    
    // ensure min heap property is maintained
    heap[0] = heap.back();
    heap.pop_back();
    heapify(0);

    return value;
  }
  
  bool empty(){
    return heap.empty();
  }

  void printHeap(){
    cout << "\tIndex |\tValue |\tPriority" << endl;
    for (int i = 0; i < heap.size(); i++){
      cout << "\t" << i << " \t" << heap[i].first 
	       << "\t" << heap[i].second << endl;
    }
  }

private:
  void heapify(int i){
    int lChildInd = LEFT(i),
        rChildInd = RIGHT(i),
        smallestInd;

    if (lChildInd < heap.size() 
		&& heap[lChildInd].second < heap[i].second){
      smallestInd = lChildInd;
    } else {
      smallestInd = i;
    }
    if (rChildInd < heap.size() 
		&& heap[rChildInd].second < heap[smallestInd].second){
      smallestInd = rChildInd;
    }
    if (i != smallestInd){
      auto p = heap[i];
      heap[i] = heap[smallestInd];
      heap[smallestInd] = p;

      heapify(smallestInd);
    }
  }

  void decreasePriority(int i, T value, float priority){
    if (priority > heap[i].second){
      cout << "PQueue error: decrease priority: "
	       << "priority greater than value in heap (" 
           << priority << " > " << heap[i].second << ")" << endl;
      return;
    }

    heap[i] = make_pair(value, priority);

    while(i > 0 && heap[PARENT(i)].second > heap[i].second){
      auto p = heap[i];
      heap[i] = heap[PARENT(i)];
      heap[PARENT(i)] = p;
      i = PARENT(i);
    }
  }
  
  vector < pair <T, float> > heap;
};

#endif